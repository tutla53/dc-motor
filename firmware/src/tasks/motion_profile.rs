/*
    Motion Profile Generator
*/

use {
    libm::sqrtf,
    {defmt_rtt as _, panic_probe as _},
};

pub struct TrapezoidProfile {
    initial_position: f32,
    target_position: f32,
    v_max: f32,
    a_max: f32,
    t_acc: f32,
    t_coast: f32,
    t_total: f32,
    profile_type: ProfileType,
    direction: f32,
}

enum ProfileType {
    Trapezoidal,
    Triangular,
}

impl TrapezoidProfile {
    pub fn new(initial_position: f32, target_position: f32, v_max: f32, a_max: f32) -> Self {
        let displacement = target_position - initial_position;
        let mut direction = 1.0;
        if displacement < 0.0 { direction = -1.0; }
        let displacement_abs = displacement.abs();
        let v_max_abs = v_max.abs();
        let a_max_abs = a_max.abs();

        let d_min = (v_max_abs * v_max_abs) / a_max_abs;
        
        let (profile_type, t_acc, t_coast, t_total) = if displacement_abs >= d_min {
            // Trapezoidal profile
            let t_acc = v_max_abs / a_max_abs;
            let t_coast = (displacement_abs - d_min) / v_max_abs;
            let t_total = 2.0 * t_acc + t_coast;
            (ProfileType::Trapezoidal, t_acc, t_coast, t_total)
        } else {
            // Triangular profile
            let t_acc = sqrtf(displacement_abs / a_max_abs);
            let t_coast = 0.0;
            let t_total = 2.0 * t_acc;
            (ProfileType::Triangular, t_acc, t_coast, t_total)
        };

        Self {
            initial_position,
            target_position,
            v_max: v_max_abs,
            a_max: a_max_abs,
            t_acc,
            t_coast,
            t_total,
            profile_type,
            direction,
        }
    }

    pub fn position(&self, t: f32) -> f32 {
        if t >= self.t_total {
            return self.target_position;
        }

        let raw_position = match self.profile_type {
            ProfileType::Trapezoidal => self.trapezoidal_position(t),
            ProfileType::Triangular => self.triangular_position(t),
        };

        self.initial_position + self.direction * raw_position
    }

    fn trapezoidal_position(&self, t: f32) -> f32 {
        if t < self.t_acc {
            0.5 * self.a_max * t * t
        } else if t < self.t_acc + self.t_coast {
            let d_acc = 0.5 * self.a_max * self.t_acc * self.t_acc;
            d_acc + self.v_max * (t - self.t_acc)
        } else {
            let t_dec = t - self.t_acc - self.t_coast;
            let d_acc_coast = 0.5 * self.a_max * self.t_acc * self.t_acc + self.v_max * self.t_coast;
            d_acc_coast + self.v_max * t_dec - 0.5 * self.a_max * t_dec *t_dec
        }
    }

    fn triangular_position(&self, t: f32) -> f32 {
        if t < self.t_acc {
            0.5 * self.a_max * t * t
        } else {
            let t_dec = t - self.t_acc;
            let d_acc = 0.5 * self.a_max * self.t_acc * self.t_acc;
            d_acc + self.a_max * self.t_acc * t_dec - 0.5 * self.a_max * t_dec * t_dec
        }
    }
}