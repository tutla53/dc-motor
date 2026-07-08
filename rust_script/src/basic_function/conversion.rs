#![allow(unused)]

use super::*;

pub struct Position {
    pub count: i32,
    pub rotation: f64,
}

impl Position {
    pub fn from_count(count: i32) -> Self {
        let rotation = (count as f64 *motor_config::ROTATION_PER_COUNT);

        Self {
            count,
            rotation
        }
    }

    pub fn from_rotation(rotation: f64) -> Self {
        let count = (rotation * motor_config::COUNT_PER_ROTATION) as i32;

        Self {
            count,
            rotation,
        }
    }
}

pub struct Speed {
    pub cps: i32,
    pub rpm: f64,
}

impl Speed {
    pub fn from_cps(cps: i32) -> Self {
        let rpm = cps as f64 * motor_config::ROTATION_PER_COUNT * 60.0;

        Self {
            cps,
            rpm
        }
    }
    
    pub fn from_rpm(rpm: f64) -> Self {
        let cps = ((rpm * motor_config::COUNT_PER_ROTATION) / 60.0 ) as i32;

        Self {  
            cps,
            rpm,
        }
    }
}

pub struct Acceleration {
    pub cps_square: i32,
}

#[allow(unused)]
impl Acceleration {
    pub fn from_cps_sq(cps_square: i32) -> Self {
        Self {
            cps_square
        }
    }
    
    pub fn from_rpm_per_sec(rpm_square: f64) -> Self {
        let cps_square = (rpm_square * motor_config::COUNT_PER_ROTATION * 60.0 ) as i32;
        Self {
            cps_square
        }
    }
}