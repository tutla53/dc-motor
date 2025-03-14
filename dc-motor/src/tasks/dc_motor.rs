/*
* DC Motor Task
*  - DC Motor Gearbox Ratio 1:4.4
*  - Encoder PPR = 11
*  - Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation
*/

use {
    crate::resources::{
        global_resources as global,
        global_resources::{
            MotorCommand, 
            MotorId,
        },
    },
    embassy_rp::{
        peripherals::PIO0,
        pio::Instance,
        pio_programs::{
            rotary_encoder::{Direction, PioEncoder},
            pwm::PioPwm,
        },
    },
    embassy_time::{Ticker, Duration, Instant, with_timeout, Timer},
    core::{
        time::Duration as CoreDuration
    },
    fixed::{types::extra::U16, FixedI32},
    {defmt_rtt as _, panic_probe as _},
};

const REFRESH_INTERVAL: u64 = 100; // in us or 10 kHz
const MAX_PWM_OUTPUT: u64 = REFRESH_INTERVAL * 1000; // in nanos
const MAX_SPEED: i32 = 1200;

#[derive(PartialEq)]
enum ControlMode {
    Speed,
    Position,
    Stop,
}

struct PIDcontrol {
    kp: FixedI32::<U16>,
    ki: FixedI32::<U16>,
    kd: FixedI32::<U16>,
    integral: FixedI32::<U16>,
    prev_error: FixedI32::<U16>,
    max_threshold: i32,
}

impl PIDcontrol {
    fn new_speed_pid() -> Self {
        Self {
            kp: FixedI32::<U16>::from_num(0.2),
            ki: FixedI32::<U16>::from_num(0.05),
            kd: FixedI32::<U16>::from_num(2.0),
            integral: FixedI32::<U16>::from_num(0),
            prev_error: FixedI32::<U16>::from_num(0),
            max_threshold: MAX_PWM_OUTPUT as i32,
        }
    }

    fn new_position_pid() -> Self {
        Self {
            kp: FixedI32::<U16>::from_num(1),
            ki: FixedI32::<U16>::from_num(0),
            kd: FixedI32::<U16>::from_num(10),
            integral: FixedI32::<U16>::from_num(0),
            prev_error: FixedI32::<U16>::from_num(0),
            max_threshold: MAX_SPEED,
        }
    }

    async fn update_pid_param(&mut self, kp: FixedI32<U16>, ki: FixedI32<U16>, kd: FixedI32<U16>){
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    fn reset(&mut self){
        self.integral = FixedI32::<U16>::from_num(0);
        self.prev_error = FixedI32::<U16>::from_num(0);
    }

    fn limit_output(&mut self, sig: i32) -> i32 {
        if sig > self.max_threshold {
            self.integral -= self.prev_error;
            return self.max_threshold;
        }
        
        if sig < -1*self.max_threshold {
            self.integral -= self.prev_error;
            return -1*self.max_threshold;
        }

        return sig;
    }

    fn compute(&mut self, error: FixedI32::<U16>) -> i32 {
        self.integral = self.integral + error;
        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig_fixed = self.kp*error + self.ki*self.integral + self.kd*derivative;
        let sig = self.limit_output(sig_fixed.to_num::<i32>());

        return sig;
    }
}

pub struct DCMotor <'d, T: Instance, const SM1: usize, const SM2: usize> {
    pwm_cw: PioPwm<'d, T, SM1>,
    pwm_ccw: PioPwm<'d, T, SM2>,
    motor_id: MotorId,
    period: u64,
    speed_control: PIDcontrol,
    position_control: PIDcontrol,
    control_mode: ControlMode,
}

impl <'d, T: Instance, const SM1: usize, const SM2: usize> DCMotor <'d, T, SM1, SM2> {
    pub fn new(pwm_cw: PioPwm<'d, T, SM1>, pwm_ccw: PioPwm<'d, T, SM2>, motor_id: MotorId) -> Self {
        Self {
            pwm_cw,
            pwm_ccw,
            motor_id,
            period: REFRESH_INTERVAL,
            speed_control: PIDcontrol::new_speed_pid(),
            position_control: PIDcontrol::new_position_pid(),
            control_mode: ControlMode::Stop,
        }
    }
    
    pub fn set_period(&mut self, period: u64) {
        self.period = period;
        self.pwm_cw.set_period(CoreDuration::from_micros(period));
        self.pwm_ccw.set_period(CoreDuration::from_micros(period));
    }

    pub fn enable(&mut self) {
        self.set_period(self.period);
        self.pwm_cw.start();
        self.pwm_ccw.start();
        self.pwm_cw.write(CoreDuration::from_nanos(0));
        self.pwm_ccw.write(CoreDuration::from_nanos(0));
    }

    pub fn move_motor(&mut self, mut pwm: i32) {
        let threshold = self.period as i32;

        if pwm > 0 {
            if pwm > threshold { pwm = threshold; }
            self.pwm_cw.write(CoreDuration::from_nanos(pwm.abs() as u64));
            self.pwm_ccw.write(CoreDuration::from_nanos(0));
        }
        else {
            if pwm < -threshold { pwm = -threshold; }
            self.pwm_cw.write(CoreDuration::from_nanos(0));
            self.pwm_ccw.write(CoreDuration::from_nanos(pwm.abs() as u64));
        }
    }
}

pub struct RotaryEncoder <'d, T: Instance, const SM: usize> {
    encoder: PioEncoder<'d, T, SM>,
    motor_id: MotorId,
    count_threshold: i32,
    timeout: u32,
}

impl <'d, T: Instance, const SM: usize> RotaryEncoder <'d, T, SM> {
    pub fn new(encoder: PioEncoder<'d, T, SM>, motor_id: MotorId) -> Self {
        Self {
            encoder,
            motor_id,
            count_threshold: 3,
            timeout: 50,
        }
    }
}

#[embassy_executor::task]
pub async fn encoder_task(mut encoder: RotaryEncoder<'static, PIO0, 0>) {
    let mut count: i32 = 0;
    let mut last_reported_count: i32 = 0;
    let mut delta_count: i32 = 0;
    let mut start = Instant::now();
    
    loop {
        match with_timeout(Duration::from_millis(encoder.timeout as u64), encoder.encoder.read()).await {
            Ok(value) => {
                match value {
                    Direction::Clockwise => {
                        count = count.saturating_add(1);
                        delta_count = delta_count.saturating_add(1);
                    },
                    Direction::CounterClockwise =>{
                        count = count.saturating_sub(1);
                        delta_count = delta_count.saturating_sub(1);
                    },
                }
            },
            Err (_) => {},
        };

        if count != last_reported_count {
            global::set_current_pos(encoder.motor_id, count).await;
            last_reported_count = count;
        }
    
        let dt = start.elapsed().as_micros().max(1);
        if delta_count.abs() >= encoder.count_threshold || dt > (encoder.timeout * 1_000) as u64 {
            let speed = (delta_count * 1_000_000)/(dt as i32);
            delta_count = 0;
            start = Instant::now();
            global::set_current_speed(encoder.motor_id, speed).await;
        }
    }
}

#[embassy_executor::task]
pub async fn motor_task(mut dc_motor: DCMotor<'static, PIO0, 1, 2>) {
    let mut ticker = Ticker::every(Duration::from_millis(5));
    dc_motor.enable();

    loop {
        let command = global::get_motor_command(dc_motor.motor_id).await;
            
        match command {
            MotorCommand::SpeedControl(commanded_speed) => {
                if dc_motor.control_mode != ControlMode::Speed {
                    dc_motor.speed_control.reset();
                    dc_motor.control_mode = ControlMode::Speed;
                }

                let current_speed = global::get_current_speed(dc_motor.motor_id).await;
                let error = FixedI32::<U16>::from_num(commanded_speed - current_speed);
                let sig = dc_motor.speed_control.compute(error);
                    
                dc_motor.move_motor(sig);
                ticker.next().await;    
            },
            MotorCommand::PositionControl(commanded_position) => {
                if dc_motor.control_mode != ControlMode::Position {
                    dc_motor.position_control.reset();
                    dc_motor.control_mode = ControlMode::Position;
                }

                let current_position = global::get_current_pos(dc_motor.motor_id).await;
                let pos_error = FixedI32::<U16>::from_num(commanded_position - current_position);
                let target_speed = dc_motor.position_control.compute(pos_error);

                let current_speed = global::get_current_speed(dc_motor.motor_id).await;
                let speed_error = FixedI32::<U16>::from_num(target_speed - current_speed);
                let sig = dc_motor.speed_control.compute(speed_error);

                dc_motor.move_motor(sig);
                ticker.next().await;
                
            },
            MotorCommand::Stop => {
                let new_kp = FixedI32::<U16>::from_num(global::get_kp(dc_motor.motor_id).await);
                let new_ki = FixedI32::<U16>::from_num(global::get_ki(dc_motor.motor_id).await);
                let new_kd = FixedI32::<U16>::from_num(global::get_kd(dc_motor.motor_id).await);

                dc_motor.move_motor(0);
                dc_motor.speed_control.reset();
                dc_motor.position_control.reset();
                dc_motor.position_control.update_pid_param(new_kp, new_ki, new_kd).await;
                dc_motor.control_mode = ControlMode::Stop;
                Timer::after(Duration::from_millis(50)).await;
                ticker.reset();
            },
        }
    }
}