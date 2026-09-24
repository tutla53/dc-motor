/*
    Motion Profile Generator
*/

use super::*;

/* --------------------------- Code -------------------------- */

pub struct MovingAverageFilter<const WINDOW: usize> {
    delta_buffer: [i32; WINDOW],
    delta_idx: usize,
    window_sum: i32,
    pub last_pos: i32,
}

impl<const WINDOW: usize> MovingAverageFilter<WINDOW> {
    pub fn new() -> Self {
        Self {
            delta_buffer: [0; WINDOW],
            delta_idx: 0,
            window_sum: 0,
            last_pos: 0,
        }
    }

    pub fn calculate_speed(&mut self, new_pos: i32) -> I16F16 {
        let delta_pos = new_pos - self.last_pos;
        self.last_pos = new_pos;

        // Moving Average Method
        self.window_sum -= self.delta_buffer[self.delta_idx];
        self.window_sum += delta_pos;
        self.delta_buffer[self.delta_idx] = delta_pos;
        self.delta_idx = (self.delta_idx + 1) % WINDOW;

        I16F16::from_num(self.window_sum)
    }
}

impl<const WINDOW: usize> Default for MovingAverageFilter<WINDOW> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_zero_padded_startup() {
        let mut filter = MovingAverageFilter::<4>::new();
        let positions = [0, 1, 2, 3, 4, 4, 5, 6, 7, 8];
        let expected_sums = [0, 1, 2, 3, 4, 3, 3, 3, 3, 4];

        for (position, expected) in positions.into_iter().zip(expected_sums) {
            assert_eq!(filter.calculate_speed(position), I16F16::from_num(expected));
        }
    }

    fn check_position_window<const WINDOW: usize>() {
        let mut filter = MovingAverageFilter::<WINDOW>::default();
        // Match firmware initialization at a nonzero encoder position.
        filter.last_pos = 1000;
        let mut positions = [1000_i32; 257];

        for sample in 1..positions.len() {
            let delta = match sample % 64 {
                0..=15 => 3,
                16..=31 => -5,
                32..=47 => 0,
                _ => (sample % 7) as i32 - 3,
            };
            positions[sample] = positions[sample - 1] + delta;
            // Independent reference: displacement across the whole window.
            let expected = positions[sample] - positions[sample.saturating_sub(WINDOW)];
            assert_eq!(
                filter.calculate_speed(positions[sample]),
                I16F16::from_num(expected),
                "window {WINDOW}, sample {sample}"
            );
        }
    }

    #[test]
    fn matches_position_window_through_reversals_stops_and_wraps() {
        check_position_window::<1>();
        check_position_window::<4>();
        check_position_window::<30>();
        check_position_window::<32>();
    }
}
