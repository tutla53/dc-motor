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

pub struct RotaryEncoder <'d, T: Instance, const SM: usize> {
    encoder: PioEncoder<'d, T, SM>,
    count_threshold: i32,
    timeout: u32,
    motor: &'static MotorState,
}

impl <'d, T: Instance, const SM: usize> RotaryEncoder <'d, T, SM> {
    pub fn new(encoder: PioEncoder<'d, T, SM>, motor: &'static MotorState) -> Self {
        Self {
            encoder,
            count_threshold: 3,
            timeout: 50,
            motor,
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
                self.motor.set_current_speed(speed).await;
            }
        }
    }    
}

#[embassy_executor::task]
pub async fn encoder_task(mut encoder: RotaryEncoder<'static, PIO0, 0>) {
    encoder.run_encoder_task().await;
}