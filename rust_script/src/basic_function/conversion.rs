#![allow(unused)]

use super::*;

pub struct Position {
    pub pulse: i32,
    pub rotation: f64,
}

impl Position {
    pub fn from_pulse(pulse: i32) -> Self {
        let rotation = (pulse as f64 * motor_config::ROTATION_PER_PULSE);

        Self { pulse, rotation }
    }

    pub fn from_rotation(rotation: f64) -> Self {
        let pulse = (rotation * motor_config::PULSE_PER_ROTATION) as i32;

        Self { pulse, rotation }
    }
}

impl std::fmt::Display for Position {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} pulse, {:.2} rotation", self.pulse, self.rotation)
    }
}

pub struct Speed {
    pub pps: i32,
    pub rpm: f64,
}

impl Speed {
    pub fn from_pps(pps: i32) -> Self {
        let rpm = pps as f64 * motor_config::ROTATION_PER_PULSE * 60.0;

        Self { pps, rpm }
    }

    pub fn from_rpm(rpm: f64) -> Self {
        let pps = ((rpm * motor_config::PULSE_PER_ROTATION) / 60.0) as i32;

        Self { pps, rpm }
    }
}

pub struct Acceleration {
    pub pps_square: i32,
}

impl Acceleration {
    pub fn from_pps_sq(pps_square: i32) -> Self {
        Self { pps_square }
    }

    pub fn from_rpm_per_sec(rpm_per_sec: f64) -> Self {
        let pps_square = (rpm_per_sec * motor_config::PULSE_PER_ROTATION / 60.0) as i32;
        Self { pps_square }
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
