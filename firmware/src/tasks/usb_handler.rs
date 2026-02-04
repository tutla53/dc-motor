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

    pub fn read_u8(&mut self) -> u8 {
        let val = self.data[self.cursor];
        self.cursor += 1;
        val
    }

    pub fn read_i32(&mut self) -> i32 {
        let val = i32::from_le_bytes(self.data[self.cursor..self.cursor + 4].try_into().unwrap_or([0; 4]));
        self.cursor += 4;
        val
    }

    pub fn read_f32(&mut self) -> f32 {
        let val = f32::from_le_bytes(self.data[self.cursor..self.cursor + 4].try_into().unwrap_or([0; 4]));
        self.cursor += 4;
        val
    }

    pub fn read_u64(&mut self) -> u64 {
        let val = u64::from_le_bytes(self.data[self.cursor..self.cursor + 8].try_into().unwrap_or([0; 8]));
        self.cursor += 8;
        val
    }

    fn select_motor(&self, id: u8) -> Option<&'static MotorHandler> {
        if id == 0 {
            return Some(&MOTOR_0);
        }
        else {
            log::info!("selected_motor is not available");
            return None 
        }
    }

    fn has_remaining(&self, needed: usize) -> bool {
        self.cursor + needed <= self.data.len()
    }

    pub async fn process_command(&mut self) {
        if self.data.len() < 2 { return; }

        let header = self.read_u8();

        if header != COMMAND_HEADER {
            return; 
        }

        let op_code = self.read_u8();

        match op_code {
            1 => {
                /* start_logger
                    time_sampling (u64) = 8
                */
                if self.has_remaining(8) {
                    let time_sampling_ms = self.read_u64();
                    LOGGER.set_logging_time_sampling(time_sampling_ms);
                    LOGGER.set_logging_state(true);
                }
            },
            2 => { 
                /* stop_logger */              
                LOGGER.set_logging_state(false);
                LOGGER.log_tx_buffer.clear();
            },

            3..=13 => {
                if !self.has_remaining(1) { return; }
                let motor_id = self.read_u8();
                
                let motor = match self.select_motor(motor_id) {
                    Some(motor) => {motor},
                    None => {
                        log::error!("Motor {} not found", motor_id);
                        return;
                    },
                };

                match op_code {
                    3 => {
                        /* move_motor_speed
                            speed (i32) = 4
                        */
                        if self.has_remaining(4) {
                            let speed = self.read_i32();
                            motor.set_motor_command(
                                MotorCommand::SpeedControl(Shape::Step(speed))
                            );
                        };
                    },
                    4 => {
                        /* move_motor_abs_pos
                            pos (i32) = 4
                        */
                        if self.has_remaining(4) {
                            let pos = self.read_i32();
                            motor.set_motor_command(
                                    MotorCommand::PositionControl(Shape::Step(pos))
                            );
                        } 
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
                        if self.has_remaining(24) {
                            let config = PosPIDConfig {
                                kp: self.read_f32(),
                                ki: self.read_f32(),
                                kd: self.read_f32(),
                                kp_speed: self.read_f32(),
                                ki_speed: self.read_f32(),
                                kd_speed: self.read_f32(),            
                            };
                            
                            motor.set_pos_pid(config).await;
                        }
                    },
                    7 => {
                        /* get_motor_pos_pid_param */
                        let pid = motor.get_pos_pid().await;

                        let mut buffer = Packet::new();            
                        buffer.push_bytes(&[COMMAND_HEADER]);
                        buffer.push_bytes(&[op_code]);
                        buffer.push_bytes(&pid.kp.to_le_bytes());
                        buffer.push_bytes(&pid.ki.to_le_bytes());
                        buffer.push_bytes(&pid.kd.to_le_bytes());
                        buffer.push_bytes(&pid.kp_speed.to_le_bytes());
                        buffer.push_bytes(&pid.ki_speed.to_le_bytes());
                        buffer.push_bytes(&pid.kd_speed.to_le_bytes());
                        let _ = CMD_CHANNEL.try_send(buffer); 
                    },
                    8 => {
                        /* set_motor_speed_pid_param
                            kp (f32) = 4
                            ki (f32) = 4
                            kd (f32) = 4
                        */
                        if self.has_remaining(12) {
                            let config = PIDConfig {
                                kp: self.read_f32(),
                                ki: self.read_f32(),
                                kd: self.read_f32(),
                            };
                            motor.set_speed_pid(config).await;
                        }
                    },
                    9 => {
                        /* get_motor_speed_pid_param */
                            let pid = motor.get_speed_pid().await;
                            let mut buffer = Packet::new();            
                            buffer.push_bytes(&[COMMAND_HEADER]);
                            buffer.push_bytes(&[op_code]);
                            buffer.push_bytes(&pid.kp.to_le_bytes());
                            buffer.push_bytes(&pid.ki.to_le_bytes());
                            buffer.push_bytes(&pid.kd.to_le_bytes());
                            let _ = CMD_CHANNEL.try_send(buffer);                         
                    },
                    10 => {
                        /* move_motor_abs_pos_trapezoid
                            target (f32) = 4
                            velocity (f32) = 4
                            acceleration (f32) = 4
                        */
                        if self.has_remaining(12) {
                            let target = self.read_f32();
                            let velocity = self.read_f32();
                            let acceleration = self.read_f32();
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
                        buffer.push_bytes(&[COMMAND_HEADER]);
                        buffer.push_bytes(&[op_code]);
                        buffer.push_bytes(&motor_pos.to_le_bytes());
                        let _ = CMD_CHANNEL.try_send(buffer);
                    },
                    12 => {
                        /* get_motor_speed */
                        let motor_speed = motor.get_current_speed();

                        let mut buffer = Packet::new();            
                        buffer.push_bytes(&[COMMAND_HEADER]);
                        buffer.push_bytes(&[op_code]);
                        buffer.push_bytes(&motor_speed.to_le_bytes());
                        let _ = CMD_CHANNEL.try_send(buffer);
                    },
                    13 => {
                        /* move_motor_open_loop
                            pwm (i32) = 4
                        */
                        if self.has_remaining(4) {
                            let pwm = self.read_i32();
                            motor.set_motor_command(
                                MotorCommand::OpenLoop(pwm)
                            );
                        }
                    },
                    _ => { let _ = log::info!("ERR: Unknown Cmd\n"); },
                }
            },
            _ => { let _ = log::info!("ERR: Unknown Cmd\n"); },
        }
    }
}