/*
* USB Handler
* Available Command:
*  - move:
*    - move start <speed: i32>
*    - move pos <position: i32>
*    - move stop
*  - log:
*    - log start <time_sampling: u64> <logmask: u32>
*    - log stop
*  - motor_pid:
*    - motor_pid get
*    - motor_pid set kp <value: f32>
*    - motor_pid set ki <value: f32>
*    - motor_pid set kd <value: f32>
*/

use {
    crate::resources::{
        global_resources::{
            MotorCommand,
            PIDConfig,
            MOTOR_0,
            LOGGER,
        },
    },
    core::str, 
    heapless::Vec,
    {defmt_rtt as _, panic_probe as _},
};

pub async fn handle_move_motor(parts: &Vec<&str, 8>) {
    //  - move start <speed: i32>
    //  - move pos <position: i32>
    //  - move stop
    
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: move <start/stop>");
        return;
    }

    match parts[1] {
        "stop" => {
            MOTOR_0.set_motor_command(MotorCommand::Stop).await;
        },
        "start" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: move start <speed>");
                return;
            }
            match parts[2].parse::<i32>() {
                Ok(speed) => {
                    MOTOR_0.set_motor_command(MotorCommand::SpeedControl(speed)).await;
                },
                Err(e) => {
                    log::info!("Invalid Speed {:?}", e);
                }
            }
        },
        "pos" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: move pos <position>");
                return;
            }
            match parts[2].parse::<i32>() {
                Ok(position) => {
                    MOTOR_0.set_motor_command(MotorCommand::PositionControl(position)).await;
                },
                Err(e) => {
                    log::info!("Invalid Position {:?}", e);
                }
            }
        },
        _ => { 
            log::info!("Invalid Parameter: move <start/stop>");
        },
    }
}

pub async fn handle_firmware_logger(parts: &Vec<&str, 8>) {
    //  - log start <time_sampling: u64> <logmask: u32>
    //  - log stop

    if parts.len() < 2 {
        log::info!("Insufficient Parameter: log <start/stop>");
        return;
    }
    
    match parts[1] {
        "start" => {
            if parts.len() < 4 {
                log::info!("Insufficient Parameter: log start <time_sampling_ms> <logmask>");
                return;
            }
            
            match parts[2].parse::<u64>() {
                Ok(time_sampling_ms) => {
                    match parts[3].parse::<u32>() {
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
        },
        "stop" => {
            LOGGER.set_logging_state(false).await;
        },
        _ => { log::info!("Invalid Parameter: log <start/stop>"); },
    }
}

pub async fn handle_motor_pid(parts: &Vec<&str, 8>) {
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: motor_pid <get/set>");
        return;
    }
    
    match parts[1] {
        "set" => {
            if parts.len() < 6 {
                log::info!("Insufficient Parameter: motor_pid set <pos/speed> <kp> <ki> <kd>");
                return;
            }

            match parts[2] {
                "speed" => {    
                    match parts[3].parse::<f32>() {
                        Ok(kp) => {
                            match parts[4].parse::<f32>() {
                                Ok(ki) => {
                                    match parts[5].parse::<f32>() {
                                        Ok(kd) => {
                                            MOTOR_0.set_speed_pid(PIDConfig{kp: kp, ki: ki, kd: kd}).await;
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
                },

                "pos" => {    
                    match parts[3].parse::<f32>() {
                        Ok(kp) => {
                            match parts[4].parse::<f32>() {
                                Ok(ki) => {
                                    match parts[5].parse::<f32>() {
                                        Ok(kd) => {
                                            MOTOR_0.set_pos_pid(PIDConfig{kp: kp, ki: ki, kd: kd}).await;
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
                },

                _ => { log::info!("Invalid Parameter: motor_pid set <pos/speed> <kp> <ki> <kd>"); },
            }
        },
        "get" => {
            let pos_pid = MOTOR_0.get_pos_pid().await;
            let speed_pid = MOTOR_0.get_speed_pid().await;
            log::info!("Pos => kp:{}, ki:{}, kd:{}", pos_pid.kp, pos_pid.ki, pos_pid.kd);
            log::info!("Speed => kp:{}, ki:{}, kd:{}", speed_pid.kp, speed_pid.ki, speed_pid.kd);
        },
        _ => { log::info!("Invalid Parameter: motor_pid <get/set>"); },
    }
}