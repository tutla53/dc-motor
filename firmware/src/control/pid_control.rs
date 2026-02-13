/*
    PID Control Calculation
*/

use super::*;

/* --------------------------- Code -------------------------- */
pub struct PIDcontrol<T: Fixed>  {
    kp: T,
    ki: T,
    kd: T,
    i_limit: T,
    integral: T,
    prev_error: T,
    max_threshold: i32,
}

impl<T: Fixed+ Neg<Output = T>> PIDcontrol<T>{
    pub fn new(config: PIDConfig, threshold: i32) -> Self {
        Self {
            kp: T::from_num(config.kp),
            ki: T::from_num(config.ki),
            kd: T::from_num(config.kd),
            i_limit: T::from_num(config.i_limit),
            integral: T::from_num(0),
            prev_error: T::from_num(0),
            max_threshold: threshold,
        }
    }

    pub fn update_pid_param(&mut self, kp: f32, ki: f32, kd: f32, i_limit: f32) {
        self.kp = T::from_num(kp);
        self.ki = T::from_num(ki);
        self.kd = T::from_num(kd);
        self.i_limit = T::from_num(i_limit);
    }

    pub fn reset(&mut self) {
        self.integral = T::from_num(0);
        self.prev_error = T::from_num(0);
    }

    pub fn compute(&mut self, error: T) -> i32 {
        let next_integral = self.integral.saturating_add(error);
        self.integral = next_integral.clamp(-self.i_limit, self.i_limit);
        
        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);

        sig.to_num::<i32>().clamp(-self.max_threshold, self.max_threshold)
    }
}