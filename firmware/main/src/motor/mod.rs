/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::resources::DEFAULT_MOTOR_CONTROL_MAX_SPEED_PPS;
use crate::resources::DEFAULT_PID_POS_CONFIG;
use crate::resources::DEFAULT_PID_SPEED_CONFIG;
use crate::resources::MOTOR_MAX_PWM_TICKS;
use crate::resources::PHYSICAL_MOTOR_MAX_SPEED_PPS;

use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicI32;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::channel::TrySendError;
use embassy_sync::mutex::Mutex;
use fixed::types::I16F16;
use fixed::types::I32F32;
use motor_control::PIDConfig;
use portable_atomic::AtomicBool as PortableAtomicBool;

/* --------------------------- Declare Modules -------------------------- */
pub mod motor_handler;

pub use motor_handler::*;
