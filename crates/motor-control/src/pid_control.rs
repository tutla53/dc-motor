/*
    PID Control Calculation
*/

use super::*;

/* --------------------------- Code -------------------------- */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PIDError {
    InvalidPidValue,
    MaxOutputOutOfRange,
}

impl fmt::Display for PIDError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPidValue => {
                write!(f, "PID configuration contains an invalid value")
            }
            Self::MaxOutputOutOfRange => {
                write!(f, "PID maximum output is out of range")
            }
        }
    }
}

impl core::error::Error for PIDError {}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy)]
pub struct PIDConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub i_limit: f32, // Symmetric for negative and positive
}

impl PIDConfig {
    pub fn is_valid_for<T: Fixed>(&self) -> bool {
        let values = [self.kp, self.ki, self.kd, self.i_limit];

        self.i_limit >= 0.0
            && values
                .iter()
                .all(|&value| value.is_finite() && T::checked_from_num(value).is_some())
    }
}

pub struct PIDController<T: Fixed> {
    kp: T,
    ki: T,
    kd: T,
    i_limit: T,
    integral: T,
    prev_error: T,
    max_output: i32,
}

impl<T: Fixed + Neg<Output = T>> PIDController<T> {
    pub fn new(config: PIDConfig, max_output: u32) -> Result<Self, PIDError> {
        if !config.is_valid_for::<T>() {
            return Err(PIDError::InvalidPidValue);
        }

        let max_output = i32::try_from(max_output).map_err(|_| PIDError::MaxOutputOutOfRange)?;

        Ok(Self {
            kp: T::from_num(config.kp),
            ki: T::from_num(config.ki),
            kd: T::from_num(config.kd),
            i_limit: T::from_num(config.i_limit),
            integral: T::from_num(0),
            prev_error: T::from_num(0),
            max_output,
        })
    }

    pub fn update_pid_param(&mut self, config: PIDConfig) -> Result<(), PIDError> {
        if !config.is_valid_for::<T>() {
            return Err(PIDError::InvalidPidValue);
        }

        self.kp = T::from_num(config.kp);
        self.ki = T::from_num(config.ki);
        self.kd = T::from_num(config.kd);
        self.i_limit = T::from_num(config.i_limit);
        self.reset();

        Ok(())
    }

    pub fn update_max_output(&mut self, max_output: u32) -> Result<(), PIDError> {
        let max_output = i32::try_from(max_output).map_err(|_| PIDError::MaxOutputOutOfRange)?;
        self.max_output = max_output;
        Ok(())
    }

    pub fn reset(&mut self) {
        self.integral = T::from_num(0);
        self.prev_error = T::from_num(0);
    }

    #[inline(always)]
    pub fn compute(&mut self, command: i32, feedback: T) -> i32 {
        let error = T::from_num(command).saturating_sub(feedback);

        let next_integral = self.integral.saturating_add(error);
        self.integral = next_integral.clamp(-self.i_limit, self.i_limit);

        let derivative = error.saturating_sub(self.prev_error);
        self.prev_error = error;

        let p = self.kp.saturating_mul(error);
        let i = self.ki.saturating_mul(self.integral);
        let d = self.kd.saturating_mul(derivative);

        let sig = p.saturating_add(i).saturating_add(d);

        sig.to_num::<i32>().clamp(-self.max_output, self.max_output)
    }
}
