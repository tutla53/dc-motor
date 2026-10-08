#![allow(unused)]
use super::*;

// All values are stored in config/motor_config.toml; build.rs preserves these names.
// Rust: use crate::config::motor_config;
//       let dt = motor_config::DT_S;
//       let kp = motor_config::DEFAULT_PID_SPEED_CONFIG.kp;

include!(concat!(env!("OUT_DIR"), "/motor_constants.rs"));
