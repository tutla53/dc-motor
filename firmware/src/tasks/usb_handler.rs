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
    
    fn has_remaining(&self, needed: usize) -> bool {
        self.cursor + needed <= self.data.len()
    }

    pub fn read_u8(&mut self) -> Option<u8> {
        let size = 1;

        if !self.has_remaining(size) {
            log::error!("Malformed packet: expected {} bytes", size);
            return None;
        }
        
        let val = self.data[self.cursor];
        self.cursor += size;
        Some(val)
    }

    pub fn read_i32(&mut self) -> Option<i32> {
        let size = 4;

        if !self.has_remaining(size) {
            log::error!("Malformed packet: expected {} bytes", size);
            return None; 
        }

        let val = i32::from_le_bytes(self.data[self.cursor..self.cursor + size].try_into().unwrap());
        self.cursor += size;
        Some(val)
    }

    pub fn read_f32(&mut self) -> Option<f32> {
        let size = 4;

        if !self.has_remaining(size) {
            log::error!("Malformed packet: expected {} bytes", size);
            return None; 
        }

        let val = f32::from_le_bytes(self.data[self.cursor..self.cursor + size].try_into().unwrap());
        self.cursor += size;

        if val.is_finite() {
            Some(val)
        } else {
            log::warn!("Received non-finite f32 (NaN/Inf). Defaulting to 0.0");
            None
        }
    }

    pub fn read_u64(&mut self) -> Option<u64> {
        let size = 8;
        
        if !self.has_remaining(size) {
            log::error!("Malformed packet: expected {} bytes", size);
            return None; 
        }

        let val = u64::from_le_bytes(self.data[self.cursor..self.cursor + size].try_into().unwrap());
        self.cursor += size;
        Some(val)
    }

    fn select_motor(&self, motor_id: Option<u8>) -> Option<&'static MotorHandler> {
        match motor_id {
            Some(id) => {
                if id == MOTOR_0.id {
                    Some(&MOTOR_0)
                }
                else {
                    log::error!("Motor {} not found", id);
                    None
                }
            },
            None => {
                log::error!("Command not found");
                None
            }
        }

    }

    pub async fn process_command(&mut self) {
        if self.data.len() < 2 { return; }
        
        let header = match self.read_u8() {
            Some(value) => {value},
            None => return,
        };

        if header != COMMAND_HEADER {
            return; 
        }

        let op_code = match self.read_u8() {
            Some(value) => {value},
            None => return,
        };

        match op_code {
            1 => {
                /* start_logger
                    time_sampling (u64) = 8
                */
                let time_sampling_ms =  match self.read_u64(){
                    Some(val) => val,
                    None => return,
                };
                    
                if time_sampling_ms == 0 {
                    // TODO: Add Error Code (PANIC! divided by zero case)
                    return;
                }
                
                LOGGER.set_logging_time_sampling(time_sampling_ms);
                LOGGER.set_logging_state(true);

            },
            2 => { 
                /* stop_logger */              
                LOGGER.set_logging_state(false);
                LOGGER.log_tx_buffer.clear();
            },

            3..=13 => {
                let motor_id = self.read_u8();
                
                let motor = match self.select_motor(motor_id) {
                    Some(motor) => motor,
                    None => return,
                };

                match op_code {
                    3 => {
                        /* move_motor_speed
                            speed (i32) = 4
                        */
                        let speed = match self.read_i32(){
                            Some(val) => val,
                            None => return,
                        };

                        motor.set_motor_command(
                            MotorCommand::SpeedControl(speed)
                        );
                    },
                    4 => {
                        /* move_motor_abs_pos
                            pos (i32) = 4
                        */
                        let pos = match self.read_i32(){
                            Some(val) => val,
                            None => return,
                        };

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
                        buffer.push_bytes(&[COMMAND_HEADER]);
                        buffer.push_bytes(&[op_code]);
                        buffer.push_bytes(&pid.kp.to_le_bytes());
                        buffer.push_bytes(&pid.ki.to_le_bytes());
                        buffer.push_bytes(&pid.kd.to_le_bytes());
                        buffer.push_bytes(&pid.kp_speed.to_le_bytes());
                        buffer.push_bytes(&pid.ki_speed.to_le_bytes());
                        buffer.push_bytes(&pid.kd_speed.to_le_bytes());
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
                            buffer.push_bytes(&[COMMAND_HEADER]);
                            buffer.push_bytes(&[op_code]);
                            buffer.push_bytes(&pid.kp.to_le_bytes());
                            buffer.push_bytes(&pid.ki.to_le_bytes());
                            buffer.push_bytes(&pid.kd.to_le_bytes());
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
                        buffer.push_bytes(&[COMMAND_HEADER]);
                        buffer.push_bytes(&[op_code]);
                        buffer.push_bytes(&motor_pos.to_le_bytes());
                        CMD_CHANNEL.send(buffer).await;
                    },
                    12 => {
                        /* get_motor_speed */
                        let motor_speed = motor.get_current_speed();

                        let mut buffer = Packet::new();            
                        buffer.push_bytes(&[COMMAND_HEADER]);
                        buffer.push_bytes(&[op_code]);
                        buffer.push_bytes(&motor_speed.to_le_bytes());
                        CMD_CHANNEL.send(buffer).await;
                    },
                    13 => {
                        /* move_motor_open_loop
                            pwm (i32) = 4
                        */
                        let pwm = match self.read_i32(){
                            Some(val) => val,
                            None => return,
                        };

                        motor.set_motor_command(MotorCommand::OpenLoop(pwm));
                    },
                    _ => { log::info!("ERR: Unknown Cmd\n"); },
                }
            },
            _ => { log::info!("ERR: Unknown Cmd\n"); },
        }
    }
}