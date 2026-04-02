/*
* Control Hub
*/
use defmt_rtt as _;
use panic_probe as _;

use crate::resources::PIDConfig;
use core::ops::Neg;
use fixed::traits::Fixed;
use libm::sqrtf;

pub mod motion_profile;
pub mod pid_control;
pub use motion_profile::*;
pub use pid_control::*;
