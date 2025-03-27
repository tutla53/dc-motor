/*
* DC Motor Task
*  - DC Motor Gearbox Ratio 1:4.4
*  - Encoder PPR = 11
*  - Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation
*/

use {
    crate::resources::{
        global_resources::{
            MotorCommand,
            Shape, 
            MotorState,
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
    libm::sqrtf,
    {defmt_rtt as _, panic_probe as _},
};

const REFRESH_INTERVAL: u64 = 1000; // in us or 10 kHz
const MAX_PWM_OUTPUT: u64 = REFRESH_INTERVAL;
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
            kp: FixedI32::<U16>::from_num(1.0),
            ki: FixedI32::<U16>::from_num(0.25),
            kd: FixedI32::<U16>::from_num(2.0),
            integral: FixedI32::<U16>::from_num(0),
            prev_error: FixedI32::<U16>::from_num(0),
            max_threshold: MAX_PWM_OUTPUT as i32,
        }
    }

    fn new_position_pid() -> Self {
        Self {
            kp: FixedI32::<U16>::from_num(5),
            ki: FixedI32::<U16>::from_num(0.001),
            kd: FixedI32::<U16>::from_num(7.5),
            integral: FixedI32::<U16>::from_num(0),
            prev_error: FixedI32::<U16>::from_num(0),
            max_threshold: MAX_SPEED,
        }
    }

    fn update_pid_param(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = FixedI32::<U16>::from_num(kp);
        self.ki = FixedI32::<U16>::from_num(ki);
        self.kd = FixedI32::<U16>::from_num(kd);
    }

    fn reset(&mut self) {
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
    period: u64,
    speed_control: PIDcontrol,
    position_control: PIDcontrol,
    control_mode: ControlMode,
    motor: &'static MotorState,
}

impl <'d, T: Instance, const SM1: usize, const SM2: usize> DCMotor <'d, T, SM1, SM2> {
    pub fn new(pwm_cw: PioPwm<'d, T, SM1>, pwm_ccw: PioPwm<'d, T, SM2>, motor: &'static MotorState) -> Self {
        Self {
            pwm_cw,
            pwm_ccw,
            period: REFRESH_INTERVAL,
            speed_control: PIDcontrol::new_speed_pid(),
            position_control: PIDcontrol::new_position_pid(),
            control_mode: ControlMode::Stop,
            motor,
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
        self.pwm_cw.write(CoreDuration::from_micros(0));
        self.pwm_ccw.write(CoreDuration::from_micros(0));
    }

    async fn update_pos_config(&mut self) {
        let current_pos = self.motor.get_current_pos().await;
        let new_pos_pid = self.motor.get_pos_pid().await;
        
        self.position_control.reset();
        self.position_control.update_pid_param(new_pos_pid.kp, new_pos_pid.ki, new_pos_pid.kd);
        self.motor.set_commanded_pos(current_pos).await;
    }

    async fn update_speed_config(&mut self) {
        let current_speed = self.motor.get_current_speed().await;
        let new_speed_pid = self.motor.get_speed_pid().await;
        
        self.speed_control.reset();
        self.speed_control.update_pid_param(new_speed_pid.kp, new_speed_pid.ki, new_speed_pid.kd);
        self.motor.set_commanded_speed(current_speed).await;
    }

    pub fn move_motor(&mut self, mut pwm: i32) {
        let threshold = self.period as i32;

        if pwm > 0 {
            if pwm > threshold { pwm = threshold; }
            self.pwm_cw.write(CoreDuration::from_micros(pwm.abs() as u64));
            self.pwm_ccw.write(CoreDuration::from_micros(0));
        }
        else {
            if pwm < -threshold { pwm = -threshold; }
            self.pwm_cw.write(CoreDuration::from_micros(0));
            self.pwm_ccw.write(CoreDuration::from_micros(pwm.abs() as u64));
        }
    }

    pub async fn run_motor_task(&mut self) {
        let mut ticker = Ticker::every(Duration::from_millis(5));
        self.enable();

        let mut start_time = Instant::now();
        let mut initial_speed = 0;
        let mut initial_pos = 0;

        loop {
            let command = self.motor.get_motor_command().await;
                
            match command {
                MotorCommand::SpeedControl(input_shape) => {
                    if self.control_mode != ControlMode::Speed {
                        self.speed_control.reset();
                        self.control_mode = ControlMode::Speed;
                        initial_speed = self.motor.get_current_speed().await;
                        start_time = Instant::now();
                    }

                    let commanded_speed = match input_shape {
                        Shape::Step(speed) => {
                            speed
                        },
                        Shape::Trapezoidal(distance, vel, acc) => {
                            let profile = TrapezoidProfile::new(initial_speed as f32, distance, vel, acc);
                            let elapsed = ((start_time.elapsed().as_millis()) as f32)/ 1000.0;
                            profile.position(elapsed) as i32
                        }
                    };

                    self.motor.set_commanded_speed(commanded_speed).await;
                    let current_speed = self.motor.get_current_speed().await;
                    let error = FixedI32::<U16>::from_num(commanded_speed - current_speed);
                    let sig = self.speed_control.compute(error);
                    self.move_motor(sig);
                    ticker.next().await;    
                },
                MotorCommand::PositionControl(input_shape) => {
                    if self.control_mode != ControlMode::Position {
                        self.position_control.reset();
                        self.control_mode = ControlMode::Position;
                        initial_pos = self.motor.get_current_pos().await;
                        start_time = Instant::now();
                    }

                    let commanded_position = match input_shape {
                        Shape::Step(position) => {
                            position
                        }
                        Shape::Trapezoidal(distance, vel, acc) => {
                            let profile = TrapezoidProfile::new(initial_pos as f32, distance, vel, acc);
                            let elapsed = ((start_time.elapsed().as_millis()) as f32)/ 1000.0;
                            profile.position(elapsed) as i32
                        }
                    };
                    
                    self.motor.set_commanded_pos(commanded_position).await;

                    let current_position = self.motor.get_current_pos().await;
                    let pos_error = FixedI32::<U16>::from_num(commanded_position - current_position);
                    let target_speed = self.position_control.compute(pos_error);

                    let current_speed = self.motor.get_current_speed().await;
                    let speed_error = FixedI32::<U16>::from_num(target_speed - current_speed);
                    let sig = self.speed_control.compute(speed_error);
                    self.move_motor(sig);
                    ticker.next().await;
                },
                MotorCommand::Stop => {
                    self.control_mode = ControlMode::Stop;
                    self.move_motor(0);
                    
                    self.update_pos_config().await;
                    self.update_speed_config().await;
                    
                    Timer::after(Duration::from_millis(50)).await;
                    start_time = Instant::now();
                    ticker.reset();
                },
            }
        }
    }
}

pub struct RotaryEncoder <'d, T: Instance, const SM: usize> {
    encoder: PioEncoder<'d, T, SM>,
    count_threshold: i32,
    timeout: u32,
    motor: &'static MotorState,
}

impl <'d, T: Instance, const SM: usize> RotaryEncoder <'d, T, SM> {
    pub fn new(encoder: PioEncoder<'d, T, SM>, motor: &'static MotorState) -> Self {
        Self {
            encoder,
            count_threshold: 3,
            timeout: 50,
            motor,
        }
    }

    pub async fn run_encoder_task(&mut self) {
        let mut count: i32 = 0;
        let mut last_reported_count: i32 = 0;
        let mut delta_count: i32 = 0;
        let mut start = Instant::now();
        
        loop {
            match with_timeout(Duration::from_millis(self.timeout as u64), self.encoder.read()).await {
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
                self.motor.set_current_pos(count).await;
                last_reported_count = count;
            }
        
            let dt = start.elapsed().as_micros().max(1);
            if delta_count.abs() >= self.count_threshold || dt > (self.timeout * 1_000) as u64 {
                let speed = (delta_count * 1_000_000)/(dt as i32);
                delta_count = 0;
                start = Instant::now();
                self.motor.set_current_speed(speed).await;
            }
        }
    }    
}

pub struct TrapezoidProfile {
    initial_position: f32,
    target_position: f32,
    v_max: f32,
    a_max: f32,
    t_acc: f32,
    t_coast: f32,
    t_total: f32,
    profile_type: ProfileType,
    direction: f32,
}

enum ProfileType {
    Trapezoidal,
    Triangular,
}

impl TrapezoidProfile {
    pub fn new(initial_position: f32, target_position: f32, v_max: f32, a_max: f32) -> Self {
        let displacement = target_position - initial_position;
        let mut direction = 1.0;
        if displacement < 0.0 { direction = -1.0; }
        let displacement_abs = displacement.abs();
        let v_max_abs = v_max.abs();
        let a_max_abs = a_max.abs();

        let d_min = (v_max_abs * v_max_abs) / a_max_abs;
        
        let (profile_type, t_acc, t_coast, t_total) = if displacement_abs >= d_min {
            // Trapezoidal profile
            let t_acc = v_max_abs / a_max_abs;
            let t_coast = (displacement_abs - d_min) / v_max_abs;
            let t_total = 2.0 * t_acc + t_coast;
            (ProfileType::Trapezoidal, t_acc, t_coast, t_total)
        } else {
            // Triangular profile
            let t_acc = sqrtf(displacement_abs / a_max_abs);
            let t_coast = 0.0;
            let t_total = 2.0 * t_acc;
            (ProfileType::Triangular, t_acc, t_coast, t_total)
        };

        Self {
            initial_position,
            target_position,
            v_max: v_max_abs,
            a_max: a_max_abs,
            t_acc,
            t_coast,
            t_total,
            profile_type,
            direction,
        }
    }

    pub fn position(&self, t: f32) -> f32 {
        if t >= self.t_total {
            return self.target_position;
        }

        let raw_position = match self.profile_type {
            ProfileType::Trapezoidal => self.trapezoidal_position(t),
            ProfileType::Triangular => self.triangular_position(t),
        };

        self.initial_position + self.direction * raw_position
    }

    fn trapezoidal_position(&self, t: f32) -> f32 {
        if t < self.t_acc {
            0.5 * self.a_max * t * t
        } else if t < self.t_acc + self.t_coast {
            let d_acc = 0.5 * self.a_max * self.t_acc * self.t_acc;
            d_acc + self.v_max * (t - self.t_acc)
        } else {
            let t_dec = t - self.t_acc - self.t_coast;
            let d_acc_coast = 0.5 * self.a_max * self.t_acc * self.t_acc + self.v_max * self.t_coast;
            d_acc_coast + self.v_max * t_dec - 0.5 * self.a_max * t_dec *t_dec
        }
    }

    fn triangular_position(&self, t: f32) -> f32 {
        if t < self.t_acc {
            0.5 * self.a_max * t * t
        } else {
            let t_dec = t - self.t_acc;
            let d_acc = 0.5 * self.a_max * self.t_acc * self.t_acc;
            d_acc + self.a_max * self.t_acc * t_dec - 0.5 * self.a_max * t_dec * t_dec
        }
    }
}

#[embassy_executor::task]
pub async fn encoder_task(mut encoder: RotaryEncoder<'static, PIO0, 0>) {
    encoder.run_encoder_task().await;
}

#[embassy_executor::task]
pub async fn motor_task(mut dc_motor: DCMotor<'static, PIO0, 1, 2>) {
    dc_motor.run_motor_task().await;
}