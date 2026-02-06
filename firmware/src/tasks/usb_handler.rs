/*
* USB Handler
*/

// Resources
use crate::resources::global_resources::MotorCommand;
use crate::resources::global_resources::Shape;
use crate::resources::global_resources::PIDConfig;
use crate::resources::global_resources::PosPIDConfig;
use crate::resources::global_resources::MotorHandler;
use crate::resources::global_resources::MOTOR_0;
use crate::resources::global_resources::LOGGER;
use crate::resources::global_resources::Packet;
use crate::resources::global_resources::CMD_CHANNEL;
use crate::resources::global_resources::COMMAND_HEADER;
use crate::resources::global_resources::FromLeBytes;

// Library
use defmt_rtt as _;
use panic_probe as _;

/* --------------------------- Code -------------------------- */
pub struct CommandHandler<'a> {
    pub data: &'a [u8],
    pub cursor: usize,
}

impl<'a> CommandHandler<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { 
            data,
            cursor: 0,
         }
    }

    pub fn read <T:FromLeBytes> (&mut self) -> Option<T> {
        let size = T::SIZE;
        let bytes = self.data.get(self.cursor..self.cursor + size)?;
        let val = T::from_le_bytes(bytes)?;
        
        self.cursor += size;
        Some(val)
    }

    pub fn read_f32(&mut self) -> Option<f32> {
        let val: f32 = self.read()?;
        if val.is_finite() {
            Some(val)
        } else {
            log::warn!("Non-finite float ignored");
            None
        }
    }

    fn select_motor(&self, motor_id: Option<u8>) -> Option<&'static MotorHandler> {
        match motor_id {
            Some(id) if id== MOTOR_0.id => { Some(&MOTOR_0) },
            Some(_) | None => {
                log::error!("Command not found");
                None
            }
        }
    }

    pub async fn process_command(&mut self) {
        let Some(header): Option<u8> = self.read() else { return };
        if header != COMMAND_HEADER { return; }
        let Some(op_code): Option<u8> = self.read() else { return };

        match op_code {
            1 => {
                /* start_logger
                    time_sampling (u64) = 8
                */
                let time_sampling_ms: u64 =  match self.read() {
                    None | Some(0) => return, // TODO: Add Error Code (PANIC! divided by zero case)
                    Some(val) => val,
                };
                
                LOGGER.set_logging_time_sampling(time_sampling_ms);
                LOGGER.set_logging_state(true);

            },
            2 => { 
                /* stop_logger */              
                LOGGER.set_logging_state(false);
                LOGGER.log_tx_buffer.clear();
            },

            3..=13 => {
                let motor_id: Option<u8> = self.read();
                
                let motor = match self.select_motor(motor_id) {
                    Some(motor) => motor,
                    None => return,
                };

                match op_code {
                    3 => {
                        /* move_motor_speed
                            speed (i32) = 4
                        */
                        let Some(speed): Option<i32> = self.read() else { return };

                        motor.set_motor_command(
                            MotorCommand::SpeedControl(speed)
                        );
                    },
                    4 => {
                        /* move_motor_abs_pos
                            pos (i32) = 4
                        */
                        let Some(pos): Option<i32> = self.read() else { return };

                        motor.set_motor_command(
                            MotorCommand::PositionControl(Shape::Step(pos))
                        );
                    },
                    5 => {
                        /* stop_motor */
                        motor.set_motor_command(MotorCommand::Stop);
                    },
                    6 => {
                        /* set_motor_pos_pid_param
                            kp (f32) = 4
                            ki (f32) = 4
                            kd (f32) = 4
                            kp_speed (f32) = 4
                            ki_speed (f32) = 4
                            kd_speed (f32) = 4
                        */

                        if let (Some(kp), Some(ki), Some(kd), Some(kp_speed), Some(ki_speed), Some(kd_speed)) = (
                            self.read_f32(), self.read_f32(), self.read_f32(),
                            self.read_f32(), self.read_f32(), self.read_f32()
                        ) {
                            let config = PosPIDConfig {
                                kp, ki, kd, kp_speed, ki_speed, kd_speed,
                            };
                            motor.set_pos_pid(config).await;
                        }
                    },
                    7 => {
                        /* get_motor_pos_pid_param */
                        let pid = motor.get_pos_pid().await;

                        let mut buffer = Packet::new();            
                        buffer.push(COMMAND_HEADER)
                            .push(op_code)
                            .push(pid.kp)
                            .push(pid.ki)
                            .push(pid.kd)
                            .push(pid.kp_speed)
                            .push(pid.ki_speed)
                            .push(pid.kd_speed);

                        CMD_CHANNEL.send(buffer).await; 
                    },
                    8 => {
                        /* set_motor_speed_pid_param
                            kp (f32) = 4
                            ki (f32) = 4
                            kd (f32) = 4
                        */
                        if let (Some(kp), Some(ki), Some(kd)) = (
                            self.read_f32(), self.read_f32(), self.read_f32(),
                        ) {
                            let config = PIDConfig {
                                kp, ki, kd,
                            };
                            motor.set_speed_pid(config).await;
                        }
                    },
                    9 => {
                        /* get_motor_speed_pid_param */
                            let pid = motor.get_speed_pid().await;
                            let mut buffer = Packet::new();            
                            buffer.push(COMMAND_HEADER)
                                .push(op_code)
                                .push(pid.kp)
                                .push(pid.ki)
                                .push(pid.kd);
                            CMD_CHANNEL.send(buffer).await;                         
                    },
                    10 => {
                        /* move_motor_abs_pos_trapezoid
                            target (f32) = 4
                            velocity (f32) = 4
                            acceleration (f32) = 4
                        */

                        if let (Some(target), Some(velocity), Some(acceleration)) = (
                            self.read_f32(), self.read_f32(), self.read_f32(),
                        ) {
                            motor.set_motor_command(
                                MotorCommand::PositionControl(
                                    Shape::Trapezoidal(target, velocity, acceleration)
                                )
                            )
                        } 
                    },
                    11 => {
                        /* get_motor_pos */
                        let motor_pos = motor.get_current_pos();

                        let mut buffer = Packet::new();            
                        buffer.push(COMMAND_HEADER)
                            .push(op_code)
                            .push(motor_pos);
                        CMD_CHANNEL.send(buffer).await;
                    },
                    12 => {
                        /* get_motor_speed */
                        let motor_speed = motor.get_current_speed();

                        let mut buffer = Packet::new();            
                        buffer.push(COMMAND_HEADER)
                            .push(op_code)
                            .push(motor_speed);
                        CMD_CHANNEL.send(buffer).await;
                    },
                    13 => {
                        /* move_motor_open_loop
                            pwm (i32) = 4
                        */
                        let Some(pwm): Option<i32> = self.read() else { return };

                        motor.set_motor_command(MotorCommand::OpenLoop(pwm));
                    },
                    _ => { log::info!("ERR: Unknown Cmd\n"); },
                }
            },
            _ => { log::info!("ERR: Unknown Cmd\n"); },
        }
    }
}