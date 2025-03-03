//! Main
// DC Motor Ratio 1:4.4
// Encoder PPR = 11
// Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation

#![no_std]
#![no_main]

use {
    embassy_executor::Spawner,
    embassy_rp::bind_interrupts,
    embassy_rp::{
        peripherals::USB,
        peripherals::PIO0,
        usb::{Driver, InterruptHandler as UsbInterruptHandler},
        pio::{InterruptHandler, Pio},
        pio_programs::{
            rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
            pwm::{PioPwmProgram, PioPwm},
        },
    },
    embassy_time::{Ticker, Duration, Instant, with_timeout, Timer},
    embassy_usb_logger::ReceiverHandler,
    embassy_sync::{
        signal::Signal,
        mutex::Mutex,
        channel::Channel,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    core::{
        str, 
        sync::atomic::{AtomicI32, Ordering}, 
        time::Duration as CoreDuration
    },
    heapless::Vec,
    {defmt_rtt as _, panic_probe as _},
};

static CURRENT_POS: AtomicI32 = AtomicI32::new(0);
static CURRENT_SPEED: AtomicI32 = AtomicI32::new(0); // count/s
static LOGGER_RUN: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(false);
static LOGGER_TIME_SAMPLING: Mutex<CriticalSectionRawMutex, u64> = Mutex::new(10);
static LOGGER_CONTROL: Channel<CriticalSectionRawMutex, LogMask, 1024> = Channel::new();
static MOTOR_CONTROL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();
const REFRESH_INTERVAL: u64 = 1000; // us

enum MotorCommand {
    MoveMotor(i16),
    Stop,
}

struct LogMask {
    dt: u64,
    motor_speed:i32,
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

struct UsbHandler;

impl ReceiverHandler for UsbHandler {
    async fn handle_data(&self, raw_data: &[u8]) {
        if let Ok(raw_data) = str::from_utf8(raw_data) {

            let parts: Vec<&str, 8> = raw_data.split_whitespace().collect();

            if !parts.is_empty() { 
                match parts[0] {
                    "motor" => { handle_move_motor(&parts); },
                    "log"  => { handle_firmware_logger(&parts).await; }
                    _ => { log::info!("Command not found"); },
                }
            }
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, UsbHandler);
}

fn handle_move_motor(parts: &Vec<&str, 8>) {
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: move <start/stop>");
        return;
    }

    match parts[1] {
        "stop" =>{
            MOTOR_CONTROL.signal(MotorCommand::Stop);
        },
        "start" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: move start <speed>");
                return;
            }
            match parts[2].parse::<i16>() {
                Ok(speed) => {
                    MOTOR_CONTROL.signal(MotorCommand::MoveMotor(speed));
                },
                Err(e) => {
                    log::info!("Invalid Speed {:?}", e);
                }
            }
        },
        _ => { 
            log::info!("Invalid Parameter: move <start/stop>");
        },
    }
}

async fn handle_firmware_logger(parts: &Vec<&str, 8>) {
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: log <start/stop>");
        return;
    }
    
    match parts[1] {
        "start" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: log start <time_sampling_ms>");
                return;
            }
            
            match parts[2].parse::<u64>() {
                Ok(time_sampling_ms) => {
                    set_logging_time_sampling(time_sampling_ms).await;
                    set_logging_state(true).await;
                },
                Err(e) => {
                    log::info!("Invalid Time Sampling {:?}", e);
                }
            } 
        },
        "stop" => {
            set_logging_state(false).await;
        },
         _ => { log::info!("Invalid Parameter: log <start/stop>"); },
    }
}

#[embassy_executor::task]
async fn encoder_0(mut encoder: PioEncoder<'static, PIO0, 0>) {
    const COUNT_THRESHOLD: i32 = 2;
    const TIMEOUT: u64 = 200;

    let mut count = 0;
    let mut delta_count: i32 = 0;
    let mut start = Instant::now();

    loop {
        match with_timeout(Duration::from_millis(TIMEOUT), encoder.read()).await {
            Ok(value) => {
                match value {
                    Direction::Clockwise => {
                        count += 1;
                        delta_count += 1;
                    },
                    Direction::CounterClockwise =>{
                        count -= 1;
                        delta_count -= 1;
                    },
                }
            },
            Err (_) => {},
        };

        let dt = start.elapsed().as_micros();
        if (delta_count.abs() >= COUNT_THRESHOLD) || (dt > TIMEOUT * 1_000) {
            let speed = delta_count  * (1_000_000 / dt as i32);
            delta_count = 0;
            start = Instant::now();
            CURRENT_SPEED.store(speed, Ordering::Relaxed);
        }

        CURRENT_POS.store(count, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn firmware_logger_task() {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut start = Instant::now();

    loop {
        let logging = is_logging_active().await;

        if logging {
            let _ = LOGGER_CONTROL.try_send(
                LogMask{
                    dt: start.elapsed().as_millis(),
                    motor_speed: CURRENT_SPEED.load(Ordering::Relaxed)
                }
            );
            ticker.next().await;
        }
        else {
            Timer::after(Duration::from_millis(200)).await;
            start = Instant::now();
            let new_time_sampling = get_logging_time_sampling().await;
            ticker = Ticker::every(Duration::from_millis(new_time_sampling));
            ticker.reset();
        }
    }
}

#[embassy_executor::task]
async fn send_logger_task() {
    loop {
        let command = LOGGER_CONTROL.receive().await;
        log::info!("{} {}", command.dt, command.motor_speed);
    }
}

async fn set_logging_state(state: bool) {
    let mut logging = LOGGER_RUN.lock().await;
    *logging = state;
}

async fn is_logging_active() -> bool {
    return *LOGGER_RUN.lock().await;
}

async fn set_logging_time_sampling(time_sampling_ms: u64) {
    let mut current = LOGGER_TIME_SAMPLING.lock().await;
    *current = time_sampling_ms;
}

async fn get_logging_time_sampling() -> u64 {
    return *LOGGER_TIME_SAMPLING.lock().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let usb_driver = Driver::new(p.USB, Irqs);

    let Pio {
        mut common, sm0, sm1, sm2, ..
    } = Pio::new(p.PIO0, Irqs);

    let enc_prg = PioEncoderProgram::new(&mut common);
    let pwm_prg = PioPwmProgram::new(&mut common);
    
    let encoder0 = PioEncoder::new(&mut common, sm0, p.PIN_6, p.PIN_7, &enc_prg);
    let mut pwm_ccw = PioPwm::new(&mut common, sm1, p.PIN_14, &pwm_prg);
    let mut pwm_cw = PioPwm::new(&mut common, sm2, p.PIN_15, &pwm_prg);

    pwm_ccw.set_period(CoreDuration::from_micros(REFRESH_INTERVAL));
    pwm_ccw.start();
    pwm_ccw.write(CoreDuration::from_micros(0));

    pwm_cw.set_period(CoreDuration::from_micros(REFRESH_INTERVAL));
    pwm_cw.start();
    pwm_cw.write(CoreDuration::from_micros(0));
    
    spawner.must_spawn(usb_logger_task(usb_driver));
    spawner.must_spawn(encoder_0(encoder0));
    spawner.must_spawn(firmware_logger_task());
    spawner.must_spawn(send_logger_task());
    
    loop {
        let command = MOTOR_CONTROL.wait().await;

        match command{
            MotorCommand::MoveMotor(speed) => {
                if !is_logging_active().await {
                    log::info!("MoveMotor {}", speed);
                }

                if speed > 0 {
                    pwm_cw.write(CoreDuration::from_micros(speed.abs() as u64));
                    pwm_ccw.write(CoreDuration::from_micros(0));
                }
                else{
                    pwm_cw.write(CoreDuration::from_micros(0));
                    pwm_ccw.write(CoreDuration::from_micros(speed.abs() as u64));
                }
            },
            MotorCommand::Stop =>{
                if !is_logging_active().await {
                    log::info!("Motor Stop");
                }
                pwm_cw.write(CoreDuration::from_micros(0));
                pwm_ccw.write(CoreDuration::from_micros(0));
            },
        }
    }
}