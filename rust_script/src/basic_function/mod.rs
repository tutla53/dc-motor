/*
* basic_function hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::basic_function::conversion::Acceleration;
use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Pwm;
use crate::basic_function::conversion::Speed;
use crate::basic_function::move_motor::Motor;
use crate::board::rpi::Pico;
use crate::config::motor_config;
use crate::config::motor_config::DEFAULT_TIMEOUT_MS;
use crate::config::motor_config::TIMEOUT_OFFSET_MS;
use crate::config::motor_config::TIMEOUT_SCALE;
use crate::logger::fwlogger::Logger;
use crate::program::macros::MutexExt;
use crate::try_lock;

/* ------------------------ Library --------------------------- */
use colored::Colorize;
use fixed::types::I32F32;
use motor_control::PIDConfig;
use motor_control::motion_profile::TrapezoidProfile;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;
use std::time::Instant;

/* --------------------- Declare Modules ---------------------- */
pub mod conversion;
pub mod move_motor;
pub mod utility;
