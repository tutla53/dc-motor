use super::*;

const ALL_FLAGS: &[(LogMask, &str)] = &[
    (LogMask::MotorPosition, "Motor_Position(rotation)"),
    (LogMask::MotorSpeed, "Motor_Speed(RPM)"),
    (LogMask::CommandedPosition, "Commanded_Position(rotation)"),
    (LogMask::CommandedSpeed, "Commanded_Speed(RPM)"),
    (LogMask::CommandedPwm, "Commanded_PWM"),
];

const LOG_CONFIG_MAP: &[(LogMask, f64, f64)] = &[
    (LogMask::MotorPosition,     motor_config::ROTATION_PER_COUNT,      0.0),
    (LogMask::MotorSpeed,        60.0 * motor_config::ROTATION_PER_COUNT, 0.0),
    (LogMask::CommandedPosition, motor_config::ROTATION_PER_COUNT,      0.0),
    (LogMask::CommandedSpeed,    60.0 * motor_config::ROTATION_PER_COUNT, 0.0),
    (LogMask::CommandedPwm,      1.0,                                   0.0),
];

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct LogMask: usize {
        const NoLog              = 0;
        const MotorPosition      = 1 << 0;
        const MotorSpeed         = 1 << 1;
        const CommandedPosition  = 1 << 2;
        const CommandedSpeed     = 1 << 3;
        const CommandedPwm       = 1 << 4;
    }
}

impl LogMask {
    pub fn get_active_names(&self) -> Vec<&'static str> {
        let mut names = vec!["Timestamp(ms)"];
        for &(flag, name) in ALL_FLAGS {
            if self.contains(flag) {
                names.push(name);
            }
        }
        names
    }

    pub fn get_scale_offset(&self) -> (f64, f64) {
        LOG_CONFIG_MAP
            .iter()
            .find(|&&(flag, _, _)| flag == *self)
            .map(|&(_, scale, offset)| (scale, offset))
            .unwrap_or((1.0, 0.0))
    }
}

