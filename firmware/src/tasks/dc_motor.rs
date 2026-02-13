/*
* DC Motor Task
*  - DC Motor Gearbox Ratio 1:4.4
*  - Encoder PPR = 11
*  - Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation
*/

use super::*;

// Resources
use crate::resources::motor_resources::Shape;
use crate::resources::motor_resources::MotorHandler;
use crate::resources::motor_resources::MotorCommand;
use crate::resources::motor_resources::ControlMode;
use crate::resources::usb_resources::EventList;
use crate::resources::EVENT_CHANNEL;
use crate::resources::POS_TOLERANCE_COUNT;
use crate::resources::SPEED_TOLERANCE_CPS;
use crate::resources::SETTLE_TICKS;
use crate::resources::SPEED_FILTER_WINDOW;

// Control
use crate::control::PIDcontrol;
use crate::control::TrapezoidProfile;

/* --------------------------- Code -------------------------- */
pub struct DCMotor <'d, T: Instance, const SM: usize> {
    pwm_cw: PwmOutput<'d>,
    pwm_ccw: PwmOutput<'d>,
    encoder: PioEncoder<'d, T, SM>,
    motor: &'static MotorHandler,
    speed_control: PIDcontrol<I16F16>,
    position_control: PIDcontrol<I32F32>,
    control_mode: ControlMode,
    motion_profile: Option<TrapezoidProfile>,
    move_done: bool,
    command_just_received: bool,
    final_target: I32F32,
    current_settle_ticks: u32,
    current_speed: I16F16,
    current_pos: I32F32,
    delta_buffer: [i32; SPEED_FILTER_WINDOW],
    delta_idx: usize,
    ticks_to_cps_rolling: I16F16,
}

impl <'d, T: Instance, const SM: usize> DCMotor <'d, T, SM> {
    pub fn new(pwm_cw: PwmOutput<'d>, pwm_ccw: PwmOutput<'d>, encoder: PioEncoder<'d, T, SM>, motor: &'static MotorHandler) -> Self {
        Self {
            pwm_cw,
            pwm_ccw,
            encoder,
            motor,            
            speed_control: PIDcontrol::new(DEFAULT_PID_SPEED_CONFIG, motor.max_pwm_ticks),
            position_control: PIDcontrol::new(DEFAULT_PID_POS_CONFIG, motor.max_speed_cps),
            control_mode: ControlMode::Stop,
            motion_profile: None, 
            move_done: false,
            command_just_received: false,
            final_target: I32F32::from_num(0),
            current_settle_ticks: 0,
            current_speed: I16F16::from_num(0),
            current_pos: I32F32::from_num(0),
            delta_buffer: [0; SPEED_FILTER_WINDOW],
            delta_idx: 0,
            ticks_to_cps_rolling: I16F16::from_num(TICKS_TO_CPS)/I16F16::from_num(SPEED_FILTER_WINDOW),
        }
    }

    async fn update_pos_pid_config(&mut self) {
        let new_pos_pid = self.motor.get_pos_pid().await;
        
        self.position_control.reset();
        self.position_control.update_pid_param(new_pos_pid.kp, new_pos_pid.ki, new_pos_pid.kd, new_pos_pid.i_limit);
    }

    async fn update_speed_pid_config(&mut self) {
        let new_speed_pid = self.motor.get_speed_pid().await;

        self.speed_control.reset();
        self.speed_control.update_pid_param(new_speed_pid.kp, new_speed_pid.ki, new_speed_pid.kd, new_speed_pid.i_limit);
    }

    pub fn move_motor(&mut self, pwm_input: i32) {

        let pwm_value = pwm_input.clamp(-1 * self.motor.max_pwm_ticks, self.motor.max_pwm_ticks);
        self.motor.set_commanded_pwm(pwm_value);

        if pwm_value > 0 {
            let _ = self.pwm_cw.set_duty_cycle(pwm_value.abs() as u16);
            let _ = self.pwm_ccw.set_duty_cycle_fully_off();
        }
        else {
            let _ = self.pwm_cw.set_duty_cycle_fully_off();
            let _ = self.pwm_ccw.set_duty_cycle(pwm_value.abs() as u16);
        }
    }

    pub async fn reset_control_mode(&mut self, control_mode: ControlMode) {
        self.control_mode = control_mode;

        // Reset Control Parameter
        self.speed_control.reset();
        self.position_control.reset();
        self.update_speed_pid_config().await;
        self.update_pos_pid_config().await;

        match self.control_mode {
            ControlMode::OpenLoop => {
                self.motor.set_commanded_pos(0); // Set Commanded to 0 means Not in Position Control
                self.motor.set_commanded_speed(0); // Set Commanded to 0 means Not in Speed Control
            },
            ControlMode::Speed => {
                self.motor.set_commanded_pos(0); // Set Commanded to 0 means Not in Position Control
            },
            ControlMode::Position => {
                self.motor.set_commanded_speed(0); // Set Commanded to 0 means Not in Speed Control
            },
            ControlMode::Stop => {
                self.move_motor(0);
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

        let mut ticker = Ticker::every(Duration::from_micros(TIME_SAMPLING_US));
        // self.enable();

        let mut start_time = Instant::now();
        
        let mut current_active_cmd = MotorCommand::Stop;
        let mut last_command: Option<MotorCommand> = None;
        let mut current_pos = self.motor.get_current_pos(); 
        let mut last_pos_for_speed = current_pos;

        self.control_mode = ControlMode::Stop;
        self.speed_control.reset();
        self.position_control.reset();
        self.update_speed_pid_config().await;
        self.update_pos_pid_config().await;

        loop {
            match select(self.encoder.read(), ticker.next()).await {
                Either::First(direction) => {
                    match direction {
                        Direction::Clockwise => current_pos = current_pos.saturating_add(1),
                        Direction::CounterClockwise => current_pos = current_pos.saturating_sub(1),
                    }
                    self.current_pos = I32F32::from_num(current_pos);
                    self.motor.set_current_pos(current_pos);
                    
                    continue; 
                }

                Either::Second(_) => {
                    let delta_pos = current_pos - last_pos_for_speed;
                    last_pos_for_speed = current_pos;

                    self.delta_buffer[self.delta_idx] = delta_pos;
                    self.delta_idx = (self.delta_idx + 1) & (SPEED_FILTER_WINDOW -1);
                    let window_sum: i32 = self.delta_buffer.iter().sum();

                    self.current_speed = I16F16::from_num(window_sum) * self.ticks_to_cps_rolling;
                    self.motor.set_current_speed(self.current_speed.to_num::<i32>());

                    if let Some(motor_command) = self.motor.get_motor_command() {

                        if Some(motor_command) != last_command {
                            last_command = Some(motor_command);
                            current_active_cmd = motor_command;
                            self.move_done = false;
                            self.current_settle_ticks = 0;
                            self.command_just_received = true;
                        
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
                                    let start_pos = self.current_pos.to_num::<f32>();
                                    start_time = Instant::now();
                                    self.motion_profile = Some(
                                        TrapezoidProfile::new(start_pos, final_pos, vel, acc)
                                    );
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
                            let error = I16F16::from_num(commanded_speed) - self.current_speed;
                            let sig = self.speed_control.compute(error);
                            self.move_motor(sig);
                        },
                        MotorCommand::PositionControl(input_shape) => {
                            let commanded_position = match input_shape {
                                Shape::Step(position) => {
                                    self.final_target = I32F32::from_num(position);
                                    position
                                },
                                Shape::Trapezoidal(target, _, _) => {
                                    if let Some(ref profile) = self.motion_profile {
                                        self.final_target = I32F32::from_num(target);
                                        let elapsed_ms = start_time.elapsed().as_millis();
                                        let elapsed_s = (elapsed_ms as f32) / 1_000.0;
                                        profile.position(elapsed_s) as i32
                                    } 
                                    else {
                                        self.current_pos.to_num::<i32>() 
                                    }
                                }
                            };
                            
                            self.motor.set_commanded_pos(commanded_position);

                            // Move Done Check
                            let at_target = (self.current_pos - self.final_target).abs() <= POS_TOLERANCE_COUNT;
                            let is_steady = self.current_speed.abs() <= SPEED_TOLERANCE_CPS;

                            if at_target && is_steady && !self.command_just_received {
                                self.current_settle_ticks += 1;
                            } else {
                                self.current_settle_ticks = 0;
                                self.command_just_received = false; 
                            } 
                            if self.current_settle_ticks >= SETTLE_TICKS && !self.move_done {
                                self.move_done = true;
                                let _ = EVENT_CHANNEL.try_send(EventList::MotorMoveDone(self.motor.id));
                            }

                            // PID Computation
                            let pos_error = I32F32::from_num(commanded_position) - self.current_pos;
                            let target_speed = self.position_control.compute(pos_error);

                            let speed_error = I16F16::from_num(target_speed) - self.current_speed;
                            let sig = self.speed_control.compute(speed_error);
                            self.move_motor(sig);
                        },
                        MotorCommand::Stop => {
                            self.motor.set_commanded_pos(self.current_pos.to_num::<i32>() );
                            self.motor.set_commanded_speed(self.current_speed.to_num::<i32>() );
                            self.move_motor(0);
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn motor0_task(mut dc_motor: DCMotor<'static, PIO0, 0>) {
    dc_motor.run_motor_task().await;
}

#[embassy_executor::task]
pub async fn motor1_task(mut dc_motor: DCMotor<'static, PIO0, 1>) {
    dc_motor.run_motor_task().await;
}