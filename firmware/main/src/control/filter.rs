/*
    Motion Profile Generator
*/

use super::*;

/* --------------------------- Code -------------------------- */

pub struct MovingAverageFilter<const WINDOW: usize> {
    delta_buffer: [i32; WINDOW],
    delta_idx: usize,
    pub last_pos: i32,
}

impl<const WINDOW: usize> MovingAverageFilter<WINDOW> {
    pub fn new() -> Self {
        Self {
            delta_buffer: [0; WINDOW],
            delta_idx: 0,
            last_pos: 0,
        }
    }

    pub fn calculate_speed(&mut self, new_pos: i32) -> I16F16 {
        let delta_pos = new_pos - self.last_pos;
        self.last_pos = new_pos;

        // Moving Average Method
        self.delta_buffer[self.delta_idx] = delta_pos;
        self.delta_idx = (self.delta_idx + 1) & (WINDOW - 1);
        let window_sum: i32 = self.delta_buffer.iter().sum();

        I16F16::from_num(window_sum)
    }
}
