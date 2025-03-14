/*
* USB Handler
* Available Command:
*  - move:
*    - move start <speed: i32>
*    - move stop
*  - log:
*    - log start <time_sampling: u64>
*    - log stop
*  - motor_pid:
*    - motor_pid get
*    - motor_pid set kp <value: f32>
*    - motor_pid set ki <value: f32>
*    - motor_pid set kd <value: f32>
*/

use {
    crate::resources::{
        global_resources as global,
        global_resources::{
            MotorCommand, 
            MotorId::Motor0,
        },
    },
    core::str, 
    heapless::Vec,
    {defmt_rtt as _, panic_probe as _},
};

pub async fn handle_move_motor(parts: &Vec<&str, 8>) {
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: move <start/stop>");
        return;
    }

    match parts[1] {
        "stop" => {
            global::set_motor_command(Motor0, MotorCommand::Stop).await;
        },
        "start" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: move start <speed>");
                return;
            }
            match parts[2].parse::<i32>() {
                Ok(speed) => {
                    global::set_motor_command(Motor0, MotorCommand::SpeedControl(speed)).await;
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
                    global::set_motor_command(Motor0, MotorCommand::PositionControl(position)).await;
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
    if parts.len() < 2 {
        log::info!("Insufficient Parameter: log <start/stop>");
        return;
    }
    
    match parts[1] {
        "start" => {
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: log start <time_sampling_ms>");
                return;
            }
            
            match parts[2].parse::<u64>() {
                Ok(time_sampling_ms) => {
                    global::set_logging_time_sampling(time_sampling_ms).await;
                    global::set_logging_state(true).await;
                },
                Err(e) => {
                    log::info!("Invalid Time Sampling {:?}", e);
                }
            } 
        },
        "stop" => {
            global::set_logging_state(false).await;
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
            if parts.len() < 3 {
                log::info!("Insufficient Parameter: motor_pid set <kp/ki/kd>");
                return;
            }
            
            match parts[2] {
                "kp" => {
                    if parts.len() < 4 {
                        log::info!("Insufficient Parameter: motor_pid set kp <value>");
                        return;
                    }
    
                    match parts[3].parse::<f32>() {
                        Ok(value) => {
                            global::set_kp(Motor0, value).await;
                        },
                        Err(e) => {
                            log::info!("Invalid kp value {:?}", e);
                        }
                    } 
                },

                "ki" => {
                    if parts.len() < 4 {
                        log::info!("Insufficient Parameter: motor_pid set ki <value>");
                        return;
                    }
    
                    match parts[3].parse::<f32>() {
                        Ok(value) => {
                            global::set_ki(Motor0, value).await;
                        },
                        Err(e) => {
                            log::info!("Invalid ki value {:?}", e);
                        }
                    } 
                },

                "kd" => {
                    if parts.len() < 4 {
                        log::info!("Insufficient Parameter: motor_pid set kd <value>");
                        return;
                    }
    
                    match parts[3].parse::<f32>() {
                        Ok(value) => {
                            global::set_kd(Motor0, value).await;
                        },
                        Err(e) => {
                            log::info!("Invalid kp value {:?}", e);
                        }
                    } 
                },
                _ => { log::info!("Invalid Parameter: motor_pid set <kp/ki/kd>"); },
            }
        },
        "get" => {
            log::info!("Kp:{}, Ki:{}, Kd:{}", global::get_kp(Motor0).await, global::get_ki(Motor0).await, global::get_kd(Motor0).await);
        },
        _ => { log::info!("Invalid Parameter: motor_pid <get/set>"); },
    }
}