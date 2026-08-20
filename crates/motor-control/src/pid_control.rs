/*
    PID Control Calculation
*/

use super::*;

/* --------------------------- Code -------------------------- */
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy)]
pub struct PIDConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub i_limit: f32, // Symmetric for negative and positive
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
    pub fn new(config: PIDConfig, max_output: i32) -> Self {
        Self {
            kp: T::from_num(config.kp),
            ki: T::from_num(config.ki),
            kd: T::from_num(config.kd),
            i_limit: T::from_num(config.i_limit),
            integral: T::from_num(0),
            prev_error: T::from_num(0),
            max_output,
        }
    }

    pub fn update_pid_param(&mut self, kp: f32, ki: f32, kd: f32, i_limit: f32) {
        self.kp = T::from_num(kp);
        self.ki = T::from_num(ki);
        self.kd = T::from_num(kd);
        self.i_limit = T::from_num(i_limit);
    }

    pub fn update_max_output(&mut self, max_output: i32) {
        self.max_output = max_output;
    }

    pub fn reset(&mut self) {
        self.integral = T::from_num(0);
        self.prev_error = T::from_num(0);
    }

    #[inline(always)]
    pub fn compute(&mut self, command: i32, feedback: T) -> i32 {
        let error = T::from_num(command) - feedback;

        let next_integral = self.integral.saturating_add(error);
        self.integral = next_integral.clamp(-self.i_limit, self.i_limit);

        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);

        sig.to_num::<i32>().clamp(-self.max_output, self.max_output)
    }
}
