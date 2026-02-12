/*
    PID Control Calculation
*/

use super::*;

/* --------------------------- Code -------------------------- */
pub struct PIDcontrol<T: Fixed>  {
    kp: T,
    ki: T,
    kd: T,
    integral: T,
    prev_error: T,
    max_threshold: i32,
}

impl<T: Fixed> PIDcontrol<T>{
    pub fn new(threshold: i32) -> Self {
        Self {
            kp: T::from_num(0),
            ki: T::from_num(0),
            kd: T::from_num(0),
            integral: T::from_num(0),
            prev_error: T::from_num(0),
            max_threshold: threshold,
        }
    }

    pub fn update_pid_param(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = T::from_num(kp);
        self.ki = T::from_num(ki);
        self.kd = T::from_num(kd);
    }

    pub fn reset(&mut self) {
        self.integral = T::from_num(0);
        self.prev_error = T::from_num(0);
    }

    pub fn compute(&mut self, error: T) -> i32 {
        self.integral = self.integral + error;
        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);
        
        if sig > self.max_threshold { self.integral -= self.prev_error; }
        if sig < -1*self.max_threshold { self.integral -= self.prev_error; }

        sig.to_num::<i32>().clamp(-self.max_threshold, self.max_threshold)
    }
}