/* 
* Control Hub
*/

use crate::resources::PIDConfig;

use defmt_rtt as _;
use panic_probe as _;
use libm::sqrtf;
use fixed::traits::Fixed;
use core::ops::Neg;


pub mod pid_control;
pub mod motion_profile;

pub use pid_control::*;
pub use motion_profile::*;

