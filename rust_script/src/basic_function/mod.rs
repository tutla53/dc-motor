use crate::SHARED;
use crate::board::rpi::Pico;
use crate::config::motor_config;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub mod move_motor;
pub mod conversion;
pub mod utility;

use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Speed;
use crate::basic_function::conversion::Acceleration;