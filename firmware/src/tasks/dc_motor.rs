/*
* DC Motor Task
*  - DC Motor Gearbox Ratio 1:4.4
*  - Encoder PPR = 11
*  - Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation
*/

use {
    crate::{
        tasks::{
            pid_control::PIDcontrol,
            motion_profile::TrapezoidProfile,
        },
        resources::{
            global_resources::{
                MotorCommand,
                Shape, 
                MotorState,
            },
        },
    },
    embassy_rp::{
        peripherals::PIO0,
        pio::Instance,
        pio_programs::pwm::PioPwm,
    },
    embassy_time::{Ticker, Duration, Instant, Timer},
    core::{
        time::Duration as CoreDuration
    },
    {defmt_rtt as _, panic_probe as _},
};

#[derive(PartialEq)]
enum ControlMode {
    Speed,
    Position,
    Stop,
}

pub struct DCMotor <'d, T: Instance, const SM1: usize, const SM2: usize> {
    pwm_cw: PioPwm<'d, T, SM1>,
    pwm_ccw: PioPwm<'d, T, SM2>,
    speed_control: PIDcontrol,
    position_control: PIDcontrol,
    speed_for_position_control: PIDcontrol, 
    control_mode: ControlMode,
    motor: &'static MotorState,
}

impl <'d, T: Instance, const SM1: usize, const SM2: usize> DCMotor <'d, T, SM1, SM2> {
    pub fn new(pwm_cw: PioPwm<'d, T, SM1>, pwm_ccw: PioPwm<'d, T, SM2>, motor: &'static MotorState) -> Self {
        Self {
            pwm_cw,
            pwm_ccw,
            speed_control: PIDcontrol::new_speed_pid(motor.get_max_pwm_output_us() as i32),
            position_control: PIDcontrol::new_position_pid(motor.get_max_speed_cps() as i32),
            speed_for_position_control: PIDcontrol::new_speed_pid(motor.get_max_pwm_output_us() as i32),
            control_mode: ControlMode::Stop,
            motor,
        }
    }
    
    pub fn set_period(&mut self, period: u64) {
        self.pwm_cw.set_period(CoreDuration::from_micros(period));
        self.pwm_ccw.set_period(CoreDuration::from_micros(period));
    }

    pub fn enable(&mut self) {
        self.set_period(self.motor.get_refresh_interval_us());
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
        
        self.speed_for_position_control.reset();
        self.speed_for_position_control.update_pid_param(new_pos_pid.kp_speed, new_pos_pid.ki_speed, new_pos_pid.kd_speed);
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
        let threshold = self.motor.get_max_pwm_output_us() as i32;

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
                    let error = commanded_speed - current_speed;
                    let sig = self.speed_control.compute(error as f32);
                    self.move_motor(sig);
                    ticker.next().await;    
                },
                MotorCommand::PositionControl(input_shape) => {
                    if self.control_mode != ControlMode::Position {
                        self.position_control.reset();
                        self.speed_for_position_control.reset();
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
                    let pos_error = commanded_position - current_position;
                    let target_speed = self.position_control.compute(pos_error as f32);

                    let current_speed = self.motor.get_current_speed().await;
                    let speed_error = target_speed - current_speed;
                    let sig = self.speed_for_position_control.compute(speed_error as f32);
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

#[embassy_executor::task]
pub async fn motor_task(mut dc_motor: DCMotor<'static, PIO0, 1, 2>) {
    dc_motor.run_motor_task().await;
}