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
        pio::{InterruptHandler, Pio, Instance},
        pio_programs::{
            rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
            pwm::{PioPwmProgram, PioPwm},
        },
    },
    embassy_time::{Ticker, Duration, Instant, with_timeout, Timer},
    embassy_usb_logger::ReceiverHandler,
    embassy_sync::{
        mutex::Mutex,
        channel::Channel,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    core::{
        str, 
        time::Duration as CoreDuration
    },
    heapless::Vec,
    fixed::{types::extra::U16, FixedI32},
    {defmt_rtt as _, panic_probe as _},
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

static CURRENT_POS: Mutex<CriticalSectionRawMutex, i32> = Mutex::new(0);
static CURRENT_SPEED: Mutex<CriticalSectionRawMutex, i32> = Mutex::new(0); // count/s
static LOGGER_RUN: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(false);
static LOGGER_TIME_SAMPLING: Mutex<CriticalSectionRawMutex, u64> = Mutex::new(10);
static LOGGER_CONTROL: Channel<CriticalSectionRawMutex, LogMask, 1024> = Channel::new();
static COMMANDED_MOTOR_SPEED: Mutex<CriticalSectionRawMutex, CommandedSpeed> = Mutex::new(CommandedSpeed::Speed(0));
static KP: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.2);
static KI: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.05);
static KD: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(2.0);

const REFRESH_INTERVAL: u64 = 1000; // us

struct LogMask {
    dt: u64,
    motor_speed:i32,
}

#[derive(Clone, Copy)]
enum CommandedSpeed {
    Speed(i32),
    Stop,
}

struct DCMotor <'d, T: Instance, const SM1: usize, const SM2: usize> {
    pwm_cw: PioPwm<'d, T, SM1>,
    pwm_ccw: PioPwm<'d, T, SM2>,
    period: u64,
    pid_control: PIDcontrol,
}

impl <'d, T: Instance, const SM1: usize, const SM2: usize> DCMotor <'d, T, SM1, SM2> {
    fn new(pwm_cw: PioPwm<'d, T, SM1>, pwm_ccw: PioPwm<'d, T, SM2>) -> Self {
        Self {
            pwm_cw,
            pwm_ccw,
            period: REFRESH_INTERVAL,
            pid_control: PIDcontrol::new(),
        }
    }
    
    fn set_period(&mut self, period: u64) {
        self.period = period;
        self.pwm_cw.set_period(CoreDuration::from_micros(period));
        self.pwm_ccw.set_period(CoreDuration::from_micros(period));
    }

    fn enable(&mut self) {
        self.set_period(self.period);
        self.pwm_cw.start();
        self.pwm_ccw.start();
        self.pwm_cw.write(CoreDuration::from_micros(0));
        self.pwm_ccw.write(CoreDuration::from_micros(0));
    }

    fn move_motor(&mut self, mut speed: i32) {
        let threshold = self.period as i32;

        if speed > 0 {
            if speed > threshold { speed = threshold; }
            self.pwm_cw.write(CoreDuration::from_micros(speed.abs() as u64));
            self.pwm_ccw.write(CoreDuration::from_micros(0));
        }
        else {

            if speed < -threshold { speed = -threshold; }
            self.pwm_cw.write(CoreDuration::from_micros(0));
            self.pwm_ccw.write(CoreDuration::from_micros(speed.abs() as u64));
        }
    }
}

struct PIDcontrol {
    // PID gains (Kp, Ki, Kd) in fixed-point
    kp: FixedI32::<U16>,
    ki: FixedI32::<U16>,
    kd: FixedI32::<U16>,
    integral: FixedI32::<U16>,
    prev_error: FixedI32::<U16>,
    threshold: i32,
}

impl PIDcontrol {
    fn new() -> Self {
        Self {
            kp: FixedI32::<U16>::from_num(0.2),
            ki: FixedI32::<U16>::from_num(0.05),
            kd: FixedI32::<U16>::from_num(2.0),
            integral: FixedI32::<U16>::from_num(0),
            prev_error: FixedI32::<U16>::from_num(0),
            threshold: REFRESH_INTERVAL as i32,
        }
    }

    async fn update_pid_param(&mut self){
        self.kp = FixedI32::<U16>::from_num(get_kp().await);
        self.ki = FixedI32::<U16>::from_num(get_ki().await);
        self.kd = FixedI32::<U16>::from_num(get_kd().await);
    }

    fn reset(&mut self){
        self.integral = FixedI32::<U16>::from_num(0);
        self.prev_error = FixedI32::<U16>::from_num(0);
    }

    fn compute(&mut self, error: FixedI32::<U16>) -> i32 {
        self.integral = self.integral + error;
        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig_fixed = self.kp*error + self.ki*self.integral + self.kd*derivative;
        let mut sig = sig_fixed.to_num::<i32>();

        if sig > self.threshold {
            sig = self.threshold;
            self.integral -= error;
        }
        else if sig < -1*self.threshold {
            sig = -1*self.threshold;
            self.integral -= error;
        }

        return sig;
    }
}

struct UsbHandler;

impl ReceiverHandler for UsbHandler {
    async fn handle_data(&self, raw_data: &[u8]) {
        if let Ok(raw_data) = str::from_utf8(raw_data) {

            let parts: Vec<&str, 8> = raw_data.split_whitespace().collect();

            if !parts.is_empty() { 
                match parts[0] {
                    "motor" => { handle_move_motor(&parts).await; },
                    "log"  => { handle_firmware_logger(&parts).await; },
                    "motor_pid" => { handle_motor_pid(&parts).await; },
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

async fn set_current_pos(pos: i32) {
    let mut current_pos = CURRENT_POS.lock().await;
    *current_pos = pos;
}

async fn get_current_pos() -> i32 {
    return *CURRENT_POS.lock().await;
}

async fn set_current_speed(speed: i32) {
    let mut current_speed = CURRENT_SPEED.lock().await;
    *current_speed = speed;
}

async fn get_current_speed() -> i32 {
    return *CURRENT_SPEED.lock().await;
}

async fn set_commanded_speed(speed: CommandedSpeed) {
    let mut current_speed = COMMANDED_MOTOR_SPEED.lock().await;
    *current_speed = speed;
}

async fn get_commanded_speed() -> CommandedSpeed {
    return *COMMANDED_MOTOR_SPEED.lock().await;
}

async fn set_kp(kp: f32) {
    let mut current_kp = KP.lock().await;
    *current_kp = kp;
}

async fn get_kp() -> f32 {
    return *KP.lock().await;
}

async fn set_ki(ki: f32) {
    let mut current_ki = KI.lock().await;
    *current_ki = ki;
}

async fn get_ki() -> f32 {
    return *KI.lock().await;
}

async fn set_kd(kd: f32) {
    let mut current_kd = KD.lock().await;
    *current_kd = kd;
}

async fn get_kd() -> f32 {
    return *KD.lock().await;
}


async fn handle_move_motor(parts: &Vec<&str, 8>) {
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: move <start/stop>");
        return;
    }

    match parts[1] {
        "stop" =>{
            log::info!("{}", get_current_pos().await);
            set_commanded_speed(CommandedSpeed::Stop).await;
        },
        "start" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: move start <speed>");
                return;
            }
            match parts[2].parse::<i32>() {
                Ok(speed) => {
                    set_commanded_speed(CommandedSpeed::Speed(speed)).await;
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

async fn handle_motor_pid(parts: &Vec<&str, 8>) {
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: motor_pid <get/set>");
        return;
    }
    
    match parts[1] {
        "set" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: motor_pid set <kp/ki/kd>");
                return;
            }
            
            match parts[2] {
                "kp" => {
                    if parts.len() < 4 {
                        log::info!("Insufficient Parameter: motor_pid set kp <value>");
                        return;
                    }
    
                    match parts[3].parse::<f32>() {
                        Ok(value) => {
                            set_kp(value).await;
                        },
                        Err(e) => {
                            log::info!("Invalid kp value {:?}", e);
                        }
                    } 
                },

                "ki" => {
                    if parts.len() < 4 {
                        log::info!("Insufficient Parameter: motor_pid set ki <value>");
                        return;
                    }
    
                    match parts[3].parse::<f32>() {
                        Ok(value) => {
                            set_ki(value).await;
                        },
                        Err(e) => {
                            log::info!("Invalid ki value {:?}", e);
                        }
                    } 
                },

                "kd" => {
                    if parts.len() < 4 {
                        log::info!("Insufficient Parameter: motor_pid set kd <value>");
                        return;
                    }
    
                    match parts[3].parse::<f32>() {
                        Ok(value) => {
                            set_kd(value).await;
                        },
                        Err(e) => {
                            log::info!("Invalid kp value {:?}", e);
                        }
                    } 
                },
                _ => { log::info!("Invalid Parameter: motor_pid set <kp/ki/kd>"); },
            }
        },
        "get" => {
            log::info!("Kp:{}, Ki:{}, Kd:{}", get_kp().await, get_ki().await, get_kd().await);
        },
         _ => { log::info!("Invalid Parameter: motor_pid <get/set>"); },
    }
}

#[embassy_executor::task]
async fn encoder_0(mut encoder: PioEncoder<'static, PIO0, 0>) {
    const COUNT_THRESHOLD: i32 = 3;
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
            set_current_speed(speed).await;
        }
        set_current_pos(count).await;
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
                    motor_speed: get_current_speed().await
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
        let commanded_speed = get_commanded_speed().await;
        match commanded_speed {
            CommandedSpeed::Speed(value) => {
                log::info!("{} {} {}", command.dt, command.motor_speed, value);
            }
            CommandedSpeed::Stop => {
                log::info!("{} {} {}", command.dt, command.motor_speed, 0);
            },
        }
    }
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
    let pwm_ccw = PioPwm::new(&mut common, sm1, p.PIN_14, &pwm_prg);
    let pwm_cw = PioPwm::new(&mut common, sm2, p.PIN_15, &pwm_prg);
    
    spawner.must_spawn(usb_logger_task(usb_driver));
    spawner.must_spawn(encoder_0(encoder0));
    spawner.must_spawn(firmware_logger_task());
    spawner.must_spawn(send_logger_task());
    
    let mut dc_motor = DCMotor::new(pwm_cw, pwm_ccw);
    dc_motor.enable();

    let mut ticker = Ticker::every(Duration::from_millis(5));

    loop {
        let command = get_commanded_speed().await;
        
        match command {
            CommandedSpeed::Speed(commanded_speed) => {
                let current_speed = get_current_speed().await;
                let error = FixedI32::<U16>::from_num(commanded_speed - current_speed);
                let sig = dc_motor.pid_control.compute(error);
                
                dc_motor.move_motor(sig);
                ticker.next().await;    
            },
            CommandedSpeed::Stop => {
                dc_motor.move_motor(0);
                dc_motor.pid_control.reset();
                dc_motor.pid_control.update_pid_param().await;
                Timer::after(Duration::from_millis(50)).await;
                ticker.reset();
            },
        }
    }
}