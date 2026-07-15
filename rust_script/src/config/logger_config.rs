use super::*;

pub const N_ENUM: usize = 5;

pub const LOG_MASK_NAMES: [&str; N_ENUM] = [
    "Motor_Position(rotation)",
    "Motor_Speed(RPM)",
    "Commanded_Position(rotation)",
    "Commanded_Speed(RPM)",
    "Commanded_PWM",
];

pub const SCALE_OFFSET_MOTOR: [[f64; 2]; N_ENUM] = [
    [motor_config::ROTATION_PER_COUNT, 0.0],
    [60.0 * motor_config::ROTATION_PER_COUNT, 0.0],
    [motor_config::ROTATION_PER_COUNT, 0.0],
    [60.0 * motor_config::ROTATION_PER_COUNT, 0.0],
    [1.0, 0.0],
];
