/*
* DC Motor Task
*  - DC Motor Gearbox Ratio 1:4.4
*  - Encoder PPR = 11
*  - Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation
*/

// Resources
use crate::resources::global_resources::Shape;
use crate::resources::global_resources::MotorHandler;
use crate::resources::global_resources::MotorCommand;
use crate::resources::global_resources::EventList;
use crate::resources::global_resources::EVENT_CHANNEL;

// Tasks
use crate::tasks::pid_control::PIDcontrol;
use crate::tasks::motion_profile::TrapezoidProfile;

// Library
use core::time::Duration as CoreDuration;
use defmt_rtt as _;
use panic_probe as _;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::Instance;
use embassy_rp::pio_programs::pwm::PioPwm;
use embassy_time::Ticker;
use embassy_time::Instant;
use embassy_time::Duration;

/* --------------------------- Code -------------------------- */
#[derive(PartialEq, Clone, Copy)]
pub enum ControlMode {
    Speed,
    Position,
    OpenLoop,
    Stop,
}

pub struct DCMotor <'d, T: Instance, const SM1: usize, const SM2: usize> {
    pwm_cw: PioPwm<'d, T, SM1>,
    pwm_ccw: PioPwm<'d, T, SM2>,
    speed_control: PIDcontrol,
    position_control: PIDcontrol,
    speed_for_position_control: PIDcontrol, 
    control_mode: ControlMode,
    motion_profile: Option<TrapezoidProfile>,
    move_done: bool,
    max_target:i32,
    commanded_speed: i32,
    commanded_pos: i32,
    motor: &'static MotorHandler,
}

impl <'d, T: Instance, const SM1: usize, const SM2: usize> DCMotor <'d, T, SM1, SM2> {
    pub fn new(pwm_cw: PioPwm<'d, T, SM1>, pwm_ccw: PioPwm<'d, T, SM2>, motor: &'static MotorHandler) -> Self {
        Self {
            pwm_cw,
            pwm_ccw,
            speed_control: PIDcontrol::new_speed_pid(motor.max_pwm_output_us as i32),
            position_control: PIDcontrol::new_position_pid(motor.max_speed_cps as i32),
            speed_for_position_control: PIDcontrol::new_speed_pid(motor.max_pwm_output_us as i32),
            control_mode: ControlMode::Stop,
            motion_profile: None, 
            move_done: false,
            max_target: 0,
            commanded_speed: 0,
            commanded_pos: 0,
            motor,
        }
    }
    
    pub fn set_period(&mut self, period: u64) {
        self.pwm_cw.set_period(CoreDuration::from_micros(period));
        self.pwm_ccw.set_period(CoreDuration::from_micros(period));
    }

    pub fn enable(&mut self) {
        self.set_period(self.motor.refresh_interval_us);
        self.pwm_cw.start();
        self.pwm_ccw.start();
        self.pwm_cw.write(CoreDuration::from_micros(0));
        self.pwm_ccw.write(CoreDuration::from_micros(0));
    }

    async fn update_pos_config(&mut self) {
        let new_pos_pid = self.motor.get_pos_pid().await;
        
        self.position_control.reset();
        self.position_control.update_pid_param(new_pos_pid.kp, new_pos_pid.ki, new_pos_pid.kd);
        
        self.speed_for_position_control.reset();
        self.speed_for_position_control.update_pid_param(new_pos_pid.kp_speed, new_pos_pid.ki_speed, new_pos_pid.kd_speed);
    }

    async fn update_speed_config(&mut self) {
        let new_speed_pid = self.motor.get_speed_pid().await;

        self.speed_control.reset();
        self.speed_control.update_pid_param(new_speed_pid.kp, new_speed_pid.ki, new_speed_pid.kd);
    }

    pub fn move_motor(&mut self, mut pwm: i32) {
        let threshold = self.motor.max_pwm_output_us as i32;

        if pwm > 0 {
            if pwm > threshold { pwm = threshold; }
            self.motor.set_commanded_pwm(pwm);
            self.pwm_cw.write(CoreDuration::from_micros(pwm.abs() as u64));
            self.pwm_ccw.write(CoreDuration::from_micros(0));
        }
        else {
            if pwm < -threshold { pwm = -threshold; }
            self.motor.set_commanded_pwm(pwm);
            self.pwm_cw.write(CoreDuration::from_micros(0));
            self.pwm_ccw.write(CoreDuration::from_micros(pwm.abs() as u64));
        }
    }

    pub async fn reset_control_mode(&mut self, control_mode: ControlMode) {
        self.control_mode = control_mode;

        match self.control_mode {
            ControlMode::OpenLoop => {
                self.speed_control.reset();
                self.position_control.reset();
                self.motor.set_commanded_pos(0); // Set Commanded to 0 means Not in Position Control
                self.motor.set_commanded_speed(0); // Set Commanded to 0 means Not in Speed Control
            },
            ControlMode::Speed => {
                self.speed_control.reset();
                self.update_speed_config().await;
                self.motor.set_commanded_pos(0); // Set Commanded to 0 means Not in Position Control
            },
            ControlMode::Position => {
                self.position_control.reset();
                self.update_pos_config().await;
                self.motor.set_commanded_speed(0); // Set Commanded to 0 means Not in Speed Control
            },
            ControlMode::Stop => {
                self.move_motor(0);
                self.update_pos_config().await;
                self.update_speed_config().await;
                self.motion_profile = None;
            },
        }
    }

    pub async fn run_motor_task(&mut self) {
        /*
            Available Mode
            1. Stop
            2. SpeedControl     -> Step Only
            3. PositionControl  -> Step
            4. PositionControl  -> Trapezoidal
        */

        let mut ticker = Ticker::every(Duration::from_millis(5));
        self.enable();

        let mut start_time = Instant::now();
        
        let mut current_active_cmd = MotorCommand::Stop;
        let mut last_command: Option<MotorCommand> = None;
        self.control_mode = ControlMode::Stop;

        loop {

            if let Some(motor_command) = self.motor.get_motor_command() {

                if Some(motor_command) != last_command {
                    last_command = Some(motor_command);
                    current_active_cmd = motor_command;
                
                    let new_control_mode = match motor_command {
                        MotorCommand::SpeedControl(_) => { ControlMode::Speed },
                        MotorCommand::OpenLoop(_) => { ControlMode::OpenLoop },
                        MotorCommand::PositionControl(_) => { ControlMode::Position },
                        MotorCommand::Stop => { ControlMode::Stop },
                    };

                    if self.control_mode != new_control_mode {
                        self.reset_control_mode(new_control_mode).await;
                    }

                    if let MotorCommand::PositionControl(shape) = current_active_cmd {
                        if let Shape::Trapezoidal(final_pos, vel, acc) = shape {
                            let start_pos = self.motor.get_current_pos();
                            start_time = Instant::now();
                            self.motion_profile = Some(TrapezoidProfile::new(start_pos as f32, final_pos, vel, acc));
                        }
                    }
                }
            }

            match current_active_cmd {
                MotorCommand::OpenLoop(sig) => {
                    self.move_motor(sig);
                },
                MotorCommand::SpeedControl(commanded_speed) => {
                    self.motor.set_commanded_speed(commanded_speed);
                    let current_speed = self.motor.get_current_speed();
                    let error = commanded_speed - current_speed;
                    let sig = self.speed_control.compute(error as f32);
                    self.move_motor(sig);
                },
                MotorCommand::PositionControl(input_shape) => {
                    let commanded_position = match input_shape {
                        Shape::Step(position) => {
                            position
                        },
                        Shape::Trapezoidal(_, _, _) => {
                            if let Some(ref profile) = self.motion_profile {
                                let elapsed_ms = start_time.elapsed().as_millis();
                                let elapsed_s = (elapsed_ms as f32) / 1_000.0;
                                profile.position(elapsed_s) as i32
                            } 
                            else {
                                self.motor.get_current_pos() 
                            }
                        }
                    };
                    
                    self.motor.set_commanded_pos(commanded_position);

                    let current_position = self.motor.get_current_pos();
                    let pos_error = commanded_position - current_position;
                    let target_speed = self.position_control.compute(pos_error as f32);

                    let current_speed = self.motor.get_current_speed();
                    let speed_error = target_speed - current_speed;
                    let sig = self.speed_for_position_control.compute(speed_error as f32);
                    self.move_motor(sig);
                },
                MotorCommand::Stop => {
                    self.motor.set_commanded_pos(self.motor.get_current_pos());
                    self.motor.set_commanded_speed(self.motor.get_current_speed());
                    self.move_motor(0);
                }
            }
            ticker.next().await;
        }
    }
}

#[embassy_executor::task]
pub async fn motor_task(mut dc_motor: DCMotor<'static, PIO0, 1, 2>) {
    dc_motor.run_motor_task().await;
}