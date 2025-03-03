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
    embassy_time::{Duration, Instant, with_timeout},
    embassy_usb_logger::ReceiverHandler,
    embassy_sync::{
        signal::Signal,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    defmt::*,
    core::{str, sync::atomic::{AtomicI32, Ordering}, time::Duration as CoreDuration},
    heapless::Vec,
    {defmt_rtt as _, panic_probe as _},
};

static CURRENT_POS: AtomicI32 = AtomicI32::new(0);
static CURRENT_SPEED: AtomicI32 = AtomicI32::new(0); // count/s
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();
const REFRESH_INTERVAL: u64 = 1000; // us

struct MotionParam {
    speed: i16,
    duration: i16,
}

enum Command {
    MoveMotor(MotionParam),
    Stop,
}

fn send_command(command: Command) {
    DRIVE_CONTROL.signal(command);
}

async fn wait_command() -> Command {
    DRIVE_CONTROL.wait().await
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, raw_data: &[u8]) {
        if let Ok(raw_data) = str::from_utf8(raw_data) {

            let parts: Vec<&str, 8> = raw_data.split_whitespace().collect();

            if !parts.is_empty() { 
                match parts[0] {
                    "move" => { handle_move_motor(&parts); },
                    "stop" => { handle_stop_motor(); },
                    _ => { log::info!("Command not found"); },
                }
            }

        }
    }

    fn new() -> Self {
        Self
    }
}

fn handle_move_motor(parts: &Vec<&str, 8>) {
    if parts.len() < 3 {
        log::info!("Insufficient Parameter: move <speed> <duration>");
        return;
    }

    match parts[1].parse::<i16>() {
        Ok(speed) => {
            match parts[2].parse::<i16>() {
                Ok(duration) => { 
                    send_command(Command::MoveMotor(MotionParam{speed, duration})); 
                },
                Err(e) => {
                    log::info!("Invalid Duration {:?}", e);
                }
            }
        },
        Err(e) => {
            log::info!("Invalid Speed {:?}", e);
        }
    }
}

fn handle_stop_motor() {
    send_command(Command::Stop);
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, Handler);
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

#[embassy_executor::main]
async fn main(spawner: Spawner){
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
    
    unwrap!(spawner.spawn(logger_task(usb_driver)));
    spawner.must_spawn(encoder_0(encoder0));
    
    loop {
        let command = wait_command().await;

        match command{
            Command::MoveMotor(motion_param) => {
                log::info!("MoveMotor {} {}", motion_param.speed, motion_param.duration);

                if motion_param.speed > 0 {
                    pwm_cw.write(CoreDuration::from_micros(motion_param.speed.abs() as u64));
                    pwm_ccw.write(CoreDuration::from_micros(0));
                }
                else{
                    pwm_cw.write(CoreDuration::from_micros(0));
                    pwm_ccw.write(CoreDuration::from_micros(motion_param.speed.abs() as u64));
                }
            },
            Command::Stop =>{
                log::info!("Stop");
                pwm_cw.write(CoreDuration::from_micros(0));
                pwm_ccw.write(CoreDuration::from_micros(0));
            },
        }
    }
}

// TODO LIST
/* async fn move_motor

*/

/* async fn logger
        let mut ticker = Ticker::every(Duration::from_millis(10));
        // let rps =  CURRENT_SPEED.load(Ordering::Relaxed)as f32 /48.4;
        // log::info!("1500, -1500, {:.4}", CURRENT_SPEED.load(Ordering::Relaxed));
        ticker.next().await;
*/
