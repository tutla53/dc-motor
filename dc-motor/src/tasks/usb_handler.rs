/*
* USB Handler
* Available Command:
    - start_logger
        - op: 1
        - args: [time_samplng: u64, logmask: u32]
    - stop_logger
        - op: 2
        - args: None
    - move_motor_speed
        - op: 3
        - args: [motor_id: u8, speed: i32]
    - move_motor_abs_pos
        - op: 4
        - args: [motor_id: u8, pos: i32]
    - stop_motor
        - op: 5
        - args: [motor_id: u8]
    - set_motor_pos_pid_param
        - op: 6
        - args: [motor_id: u8, kp: f32, ki: f32, kd:f32]  
    - get_motor_pos_pid_param
        - op: 7
        - args: [motor_id: u8]
        - return: [kp: f32, ki: f32, kd:f32]
    - set_motor_speed_pid_param
        - op: 8
        - args: [motor_id: u8, kp: f32, ki: f32, kd:f32]  
    - get_motor_speed_pid_param
        - op: 9
        - args: [motor_id: u8]
        - return: [kp: f32, ki: f32, kd:f32]
*/

use {
    crate::resources::{
        global_resources::{
            MotorCommand,
            PIDConfig,
            MotorState,
            MOTOR_0,
            LOGGER,
        },
    },
    core::str, 
    {defmt_rtt as _, panic_probe as _},
};

pub struct CommandHandler<'a> {
    pub parts: &'a [&'a str],
}

impl<'a> CommandHandler<'a> {
    pub fn new(parts: &'a [&'a str]) -> Self {
        Self { parts }
    }

    pub async fn process_command(&self) {
        match self.parts[0] {
            "1" => { self.start_logger().await; },
            "2" => { self.stop_logger().await; },
            "3" => { self.move_motor_speed().await; },
            "4" => { self.move_motor_abs_pos().await; },
            "5" => { self.stop_motor().await; },
            "6" => { self.set_motor_pos_pid_param().await; },
            "7" => { self.get_motor_pos_pid_param().await; },
            "8" => { self.set_motor_speed_pid_param().await; }
            "9" => { self.get_motor_speed_pid_param(). await;}
            _ => { log::info!("Command not found"); },
        }
    }
    
    fn get_motor_id(&self, idx: usize) -> Option<&'static MotorState> {
        if self.parts.len() <= idx {
            log::info!("Invalid Motor ID");
            return None;
        }

        match self.parts[idx].parse::<u8>() {
            Ok(motor_id) => {
                if motor_id == 0 { return Some(&MOTOR_0); }
                else { 
                    log::info!("Motor is not available");
                    return None 
                };
            }
            Err(_) => {
                log::info!("Invalid Motor ID");
                return None;
            }
        }
    }

    async fn start_logger(&self) {
        if self.parts.len() < 3 {
            log::info!("Insufficient Argument(s): [time_sampling: u64, logmask: u32]");
            return;
        }
    
        match self.parts[1].parse::<u64>() {
            Ok(time_sampling_ms) => {
                match self.parts[2].parse::<u32>() {
                    Ok(log_mask) => {
                        LOGGER.set_logging_time_sampling(time_sampling_ms).await;
                        LOGGER.set_log_mask(log_mask).await;
                        LOGGER.set_logging_state(true).await;
                    },
                    Err(e) => {
                        log::info!("Invalid Log Mask {:?}", e);
                    }
                }
            },
            Err(e) => {
                log::info!("Invalid Time Sampling {:?}", e);
            }
        } 
    }

    async fn stop_logger(&self) {
        LOGGER.set_logging_state(false).await;
    }

    async fn move_motor_speed(&self) {
        if self.parts.len() < 3 {
            log::info!("Insufficient Argument(s): [motor_id: u8, speed: i32]");
            return;
        }
        
        if let Some(motor_id) = self.get_motor_id(1) {
            match self.parts[2].parse::<i32>() {
                Ok(speed) => { motor_id.set_motor_command(MotorCommand::SpeedControl(speed)).await; },
                Err(e) => { log::info!("Invalid Motor Speed: {:?}", e); }
            } 
        }
    }

    async fn move_motor_abs_pos(&self) {
        if self.parts.len() < 3 {
            log::info!("Insufficient Argument(s): [motor_id: u8, pos: i32]");
            return;
        }
        
        if let Some(motor_id) = self.get_motor_id(1) {
            match self.parts[2].parse::<i32>() {
                Ok(pos) => { motor_id.set_motor_command(MotorCommand::PositionControl(pos)).await; },
                Err(e) => { log::info!("Invalid Motor Speed: {:?}", e); }
            } 
        }
    }

    async fn stop_motor(&self) {
        if self.parts.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        if let Some(motor_id) = self.get_motor_id(1) {
            motor_id.set_motor_command(MotorCommand::Stop).await;
        }
    }

    async fn set_motor_pos_pid_param(&self) {
        if self.parts.len() < 5 {
            log::info!("Insufficient Argument(s): [motor_id: u8, kp: f32, ki: f32, kd:f32]");
            return;
        }

        if let Some(motor_id) = self.get_motor_id(1) {
            match self.parts[2].parse::<f32>() {
                Ok(kp) => {
                    match self.parts[3].parse::<f32>() {
                        Ok(ki) => {
                            match self.parts[4].parse::<f32>() {
                                Ok(kd) => {
                                    motor_id.set_pos_pid(PIDConfig{kp: kp, ki: ki, kd: kd}).await;
                                },
                                Err(e) => {
                                    log::info!("Invalid kp value {:?}", e);
                                }
                            } 
                        },

                        Err(e) => {
                            log::info!("Invalid ki value {:?}", e);
                        }
                    } 
                },
                Err(e) => {
                    log::info!("Invalid kp value {:?}", e);
                }
            }
        }
    }

    async fn get_motor_pos_pid_param(&self) {
        if self.parts.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        if let Some(motor_id) = self.get_motor_id(1) {
            let pid = motor_id.get_pos_pid().await;
            log::info!("Pos => kp:{}, ki:{}, kd:{}", pid.kp, pid.ki, pid.kd);
        }
    }

    async fn set_motor_speed_pid_param(&self) {
        if self.parts.len() < 5 {
            log::info!("Insufficient Argument(s): [motor_id: u8, kp: f32, ki: f32, kd:f32]");
            return;
        }

        if let Some(motor_id) = self.get_motor_id(1) {
            match self.parts[2].parse::<f32>() {
                Ok(kp) => {
                    match self.parts[3].parse::<f32>() {
                        Ok(ki) => {
                            match self.parts[4].parse::<f32>() {
                                Ok(kd) => {
                                    motor_id.set_speed_pid(PIDConfig{kp: kp, ki: ki, kd: kd}).await;
                                },
                                Err(e) => {
                                    log::info!("Invalid kp value {:?}", e);
                                }
                            } 
                        },

                        Err(e) => {
                            log::info!("Invalid ki value {:?}", e);
                        }
                    } 
                },
                Err(e) => {
                    log::info!("Invalid kp value {:?}", e);
                }
            }
        }
    }

    async fn get_motor_speed_pid_param(&self) {
        if self.parts.len() < 2 {
            log::info!("Insufficient Argument(s): [motor_id: u8]");
            return;
        }

        if let Some(motor_id) = self.get_motor_id(1) {
            let pid = motor_id.get_speed_pid().await;
            log::info!("Speed => kp:{}, ki:{}, kd:{}", pid.kp, pid.ki, pid.kd);
        }
    }

}