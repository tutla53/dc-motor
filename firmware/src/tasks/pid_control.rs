/*
    Encoder Task
*/

use {
    crate::tasks::dc_motor::{
        MAX_PWM_OUTPUT,
        MAX_SPEED,
    },
    {defmt_rtt as _, panic_probe as _},
};

pub struct PIDcontrol {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    max_threshold: i32,
}

impl PIDcontrol {
    pub fn new_speed_pid() -> Self {
        Self {
            kp: 1.0,
            ki: 0.25,
            kd: 2.0,
            integral: 0.0,
            prev_error: 0.0,
            max_threshold: MAX_PWM_OUTPUT as i32,
        }
    }

    pub fn new_position_pid() -> Self {
        Self {
            kp: 5.0,
            ki: 0.001,
            kd: 7.5,
            integral: 0.0,
            prev_error: 0.0,
            max_threshold: MAX_SPEED,
        }
    }

    pub fn update_pid_param(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    pub fn limit_output(&mut self, sig: i32) -> i32 {
        if sig > self.max_threshold {
            self.integral -= self.prev_error;
            return self.max_threshold;
        }
        
        if sig < -1*self.max_threshold {
            self.integral -= self.prev_error;
            return -1*self.max_threshold;
        }

        return sig;
    }

    pub fn compute(&mut self, error: f32) -> i32 {
        self.integral = self.integral + error;
        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig_fixed = self.kp*error + self.ki*self.integral + self.kd*derivative;
        let sig = self.limit_output(sig_fixed as i32);

        return sig;
    }
}