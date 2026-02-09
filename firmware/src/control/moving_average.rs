/*
    Moving Average Filter
*/

pub struct MovingAverage<const N: usize> {
    buffer: [i32; N],
    index: usize,
    is_filled: bool,
    sum: i32,
}

impl<const N: usize> MovingAverage<N> {
    pub fn new() -> Self {
        Self {
            buffer: [0; N],
            index: 0,
            is_filled: false,
            sum: 0,
        }
    }

    pub fn update(&mut self, value: i32) -> i32 {
        if self.is_filled {
            self.sum -= self.buffer[self.index];
        }
        
        self.buffer[self.index] = value;
        self.sum += value;
        
        self.index = (self.index + 1) % N;
        
        if !self.is_filled && self.index == 0 {
            self.is_filled = true;
        }

        if self.is_filled {
            self.sum / N as i32
        } else {
            self.sum / self.index as i32
        }
    }
}