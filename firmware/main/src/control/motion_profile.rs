/*
    Motion Profile Generator
*/

use super::*;

/* --------------------------- Code -------------------------- */
pub struct TrapezoidProfile<T: FixedSigned> {
    initial_position: T,
    target_position: T,
    v_max: T,
    a_max: T,
    t_acc: T,
    t_coast: T,
    t_total: T,
    profile_type: ProfileType,
    direction: T,
}

enum ProfileType {
    Trapezoidal,
    Triangular,
}

impl<T: FixedSigned + FastSqrt> TrapezoidProfile<T> {
    pub fn new(initial_position: T, target_position: T, v_max: T, a_max: T) -> Result<Self, ()> {
        let displacement = target_position - initial_position;

        let direction = if displacement >= T::from_num(0.0) {
            T::from_num(1.0)
        } else {
            T::from_num(-1.0)
        };

        let displacement_abs = displacement.abs();
        let v_max_abs = v_max.abs();
        let a_max_abs = a_max.abs();

        let d_min = v_max_abs
            .saturating_mul(v_max_abs)
            .checked_div(a_max_abs)
            .ok_or(())?;

        let (profile_type, t_acc, t_coast, t_total) = if displacement_abs >= d_min {
            // Trapezoidal profile
            let t_acc = v_max_abs.checked_div(a_max_abs).ok_or(())?;
            let t_coast = (displacement_abs - d_min)
                .checked_div(v_max_abs)
                .ok_or(())?;
            let t_total = T::from_num(2.0) * t_acc + t_coast;
            (ProfileType::Trapezoidal, t_acc, t_coast, t_total)
        } else {
            // Triangular profile
            let t_acc = (displacement_abs.checked_div(a_max_abs).ok_or(())?).sqrt();
            let t_coast = T::from_num(0.0);
            let t_total = T::from_num(2.0) * t_acc;
            (ProfileType::Triangular, t_acc, t_coast, t_total)
        };

        Ok(Self {
            initial_position,
            target_position,
            v_max: v_max_abs,
            a_max: a_max_abs,
            t_acc,
            t_coast,
            t_total,
            profile_type,
            direction,
        })
    }

    pub fn position(&self, t_fixed: T) -> T {
        if t_fixed >= self.t_total {
            return self.target_position;
        }

        let raw_position = match self.profile_type {
            ProfileType::Trapezoidal => self.trapezoidal_position(t_fixed),
            ProfileType::Triangular => self.triangular_position(t_fixed),
        };

        self.initial_position + self.direction * raw_position
    }

    fn trapezoidal_position(&self, t_fixed: T) -> T {
        if t_fixed < self.t_acc {
            T::from_num(0.5) * self.a_max * t_fixed * t_fixed
        } else if t_fixed < self.t_acc + self.t_coast {
            let d_acc = T::from_num(0.5) * self.a_max * self.t_acc * self.t_acc;
            d_acc + self.v_max * (t_fixed - self.t_acc)
        } else {
            let t_dec = t_fixed - self.t_acc - self.t_coast;
            let d_acc_coast =
                T::from_num(0.5) * self.a_max * self.t_acc * self.t_acc + self.v_max * self.t_coast;
            d_acc_coast + self.v_max * t_dec - T::from_num(0.5) * self.a_max * t_dec * t_dec
        }
    }

    fn triangular_position(&self, t_fixed: T) -> T {
        if t_fixed < self.t_acc {
            T::from_num(0.5) * self.a_max * t_fixed * t_fixed
        } else {
            let t_dec = t_fixed - self.t_acc;
            let d_acc = T::from_num(0.5) * self.a_max * self.t_acc * self.t_acc;
            d_acc + self.a_max * self.t_acc * t_dec - T::from_num(0.5) * self.a_max * t_dec * t_dec
        }
    }
}
