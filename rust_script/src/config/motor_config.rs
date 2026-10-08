#![allow(unused)]
use super::*;

// All values are stored in config/motor_config.toml; build.rs preserves these names.
include!(concat!(env!("OUT_DIR"), "/motor_constants.rs"));
