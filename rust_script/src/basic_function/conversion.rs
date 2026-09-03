#![allow(unused)]

use super::*;

pub struct Position {
    pub count: i32,
    pub rotation: f64,
}

impl Position {
    pub fn from_count(count: i32) -> Self {
        let rotation = (count as f64 * motor_config::ROTATION_PER_COUNT);

        Self { count, rotation }
    }

    pub fn from_rotation(rotation: f64) -> Self {
        let count = (rotation * motor_config::COUNT_PER_ROTATION) as i32;

        Self { count, rotation }
    }
}

pub struct Speed {
    pub cps: i32,
    pub rpm: f64,
}

impl Speed {
    pub fn from_cps(cps: i32) -> Self {
        let rpm = cps as f64 * motor_config::ROTATION_PER_COUNT * 60.0;

        Self { cps, rpm }
    }

    pub fn from_rpm(rpm: f64) -> Self {
        let cps = ((rpm * motor_config::COUNT_PER_ROTATION) / 60.0) as i32;

        Self { cps, rpm }
    }
}

pub struct Acceleration {
    pub cps_square: i32,
}

impl Acceleration {
    pub fn from_cps_sq(cps_square: i32) -> Self {
        Self { cps_square }
    }

    pub fn from_rpm_per_sec(rpm_per_sec: f64) -> Self {
        let cps_square = (rpm_per_sec * motor_config::COUNT_PER_ROTATION / 60.0) as i32;
        Self { cps_square }
    }
}

pub struct Pwm {
    pub ticks: i32,
    pub percent: f64,
}

impl Pwm {
    pub fn from_ticks(ticks: i32) -> Self {
        let max_ticks = motor_config::MAX_PWM_TICKS as i32;
        let ticks = ticks.clamp(-max_ticks, max_ticks);
        let percent = ticks as f64 / max_ticks as f64 * 100.0;

        Self { ticks, percent }
    }

    pub fn from_percent(percent: f64) -> Self {
        let percent = percent.clamp(-100.0, 100.0);
        let ticks = (((percent * motor_config::MAX_PWM_TICKS as f64) / 100.0) as i32).clamp(
            -(motor_config::MAX_PWM_TICKS as i32),
            motor_config::MAX_PWM_TICKS as i32,
        );

        Self { ticks, percent }
    }
}
