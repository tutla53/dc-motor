#![allow(unused)]
pub const MOTOR_ID: u8 = 0;

// Mechanical Properties
pub const GEAR_RATIO: f64 = 4.4;
pub const ENCODER_PPR: f64 = 11.0;
pub const ROTATION_PER_COUNT: f64 = 1.0 / (GEAR_RATIO * ENCODER_PPR);
pub const COUNT_PER_ROTATION: f64 = (GEAR_RATIO * ENCODER_PPR);
pub const MAX_SPEED_PPS: i32 = 887;
pub const MAX_SPEED_RPM: f64 = MAX_SPEED_PPS as f64 * ROTATION_PER_COUNT * 60.0;

//  Electronic Properties
pub const SYSTEM_FREQ_HZ: u32 = 133_000_000;
pub const PWM_FREQ_HZ: u32 = 25_000;
pub const MAX_PWM_TICKS: i32 = ((SYSTEM_FREQ_HZ / PWM_FREQ_HZ) - 1) as i32;

// System Properties Based on the System Identification
pub const FREQUENCY_SAMPLING_HZ: u32 = 1000;
pub const DT_S: f64 = 1.0 / FREQUENCY_SAMPLING_HZ as f64;

// Linear Model Properties
pub const K_POSITIVE: f64 = 0.2058678594254962; // (pulse per seconds)/PWM_TICKS
pub const K_NEGATIVE: f64 = 0.19629188012626667; // (pulse per seconds)/PWM_TICKS
pub const TAU_S: f64 = 0.026508300557422464; // seconds
pub const L_S: f64 = 0.013976871626348452; // Delay Time (seconds)
pub const L_STEPS: i32 = (L_S / DT_S) as i32; // Delay Time (steps)
