/*
    Encoder Task
*/

use {
    crate::resources::{
        global_resources::MotorState,
    },
    embassy_rp::{
        peripherals::PIO0,
        pio::Instance,
        pio_programs::{
            rotary_encoder::{Direction, PioEncoder},
        },
    },
    embassy_time::{Duration, Instant, with_timeout},
    {defmt_rtt as _, panic_probe as _},
};

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

pub struct RotaryEncoder <'d, T: Instance, const SM: usize, const N: usize> {
    encoder: PioEncoder<'d, T, SM>,
    count_threshold: i32,
    timeout: u32,
    motor: &'static MotorState,
    filter: MovingAverage<N>,
}

impl <'d, T: Instance, const SM: usize, const N: usize> RotaryEncoder <'d, T, SM, N> {
    pub fn new(encoder: PioEncoder<'d, T, SM>, motor: &'static MotorState, filter: MovingAverage::<N>) -> Self {
        Self {
            encoder,
            count_threshold: 3,
            timeout: 50,
            motor,
            filter,
        }
    }

    pub async fn run_encoder_task(&mut self) {
        let mut count: i32 = 0;
        let mut last_reported_count: i32 = 0;
        let mut delta_count: i32 = 0;
        let mut start = Instant::now();
        
        loop {
            match with_timeout(Duration::from_millis(self.timeout as u64), self.encoder.read()).await {
                Ok(value) => {
                    match value {
                        Direction::Clockwise => {
                            count = count.saturating_add(1);
                            delta_count = delta_count.saturating_add(1);
                        },
                        Direction::CounterClockwise =>{
                            count = count.saturating_sub(1);
                            delta_count = delta_count.saturating_sub(1);
                        },
                    }
                },
                Err (_) => {},
            };
    
            if count != last_reported_count {
                self.motor.set_current_pos(count).await;
                last_reported_count = count;
            }
        
            let dt = start.elapsed().as_micros().max(1);
            if delta_count.abs() >= self.count_threshold || dt > (self.timeout * 1_000) as u64 {
                let speed = (delta_count * 1_000_000)/(dt as i32);
                delta_count = 0;
                start = Instant::now();
                let filtered_speed = self.filter.update(speed);
                self.motor.set_current_speed(filtered_speed).await;
            }
        }
    }    
}

#[embassy_executor::task]
pub async fn encoder_task(mut encoder: RotaryEncoder<'static, PIO0, 0, 10>) {
    encoder.run_encoder_task().await;
}