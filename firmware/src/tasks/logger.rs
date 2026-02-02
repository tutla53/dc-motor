/*
* Logger Task
*/

// Resources
use crate::resources::global_resources::MOTOR_0;
use crate::resources::global_resources::LOGGER;

// Library
use core::fmt::Write;
use defmt_rtt as _;
use panic_probe as _;
use embassy_time::Ticker;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Timer;

/* --------------------------- Code -------------------------- */
pub struct LogData {
    dt: u64,
    values: [i32; 5],
}

impl LogData {
    async fn select(dt: u64) -> Self {
        Self {
            dt,
            values: [
                MOTOR_0.get_current_pos().await,
                MOTOR_0.get_current_speed().await,
                MOTOR_0.get_commanded_pos().await,
                MOTOR_0.get_commanded_speed().await,
                MOTOR_0.get_commanded_pwm().await,
            ]         
        }
    }
    
    fn write_to_buffer(&self, buffer: &mut heapless::String<1024>) {
        write!(buffer, " {}", self.dt).unwrap();

        for i in 0..5 {
            write!(buffer, " {}", self.values[i]).unwrap();
        }
    }
}

#[embassy_executor::task]
pub async fn firmware_logger_task() {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut start = Instant::now();

    loop {
        let logging = LOGGER.is_logging_active().await;

        if logging {
            let dt = start.elapsed().as_millis();
            
            let data = LogData::select(dt).await;

            LOGGER.add_to_buffer(data);
            ticker.next().await;
        }
        else {
            Timer::after(Duration::from_millis(200)).await;
            start = Instant::now();
            let new_time_sampling = LOGGER.get_logging_time_sampling().await;
            ticker = Ticker::every(Duration::from_millis(new_time_sampling));
            ticker.reset();
        }
    }
}

#[embassy_executor::task]
pub async fn send_logger_task() {
    let mut buffer = heapless::String::<1024>::new();

    loop {
        let command = LOGGER.wait_for_log_request().await;

        if command {
            let batch_size = LOGGER.buffer_len();

            if batch_size > 0 {
                for _ in 0..batch_size {  
                    let data = LOGGER.get_buffered_data().await;
                    data.write_to_buffer(&mut buffer);
                }

                log::info!("log {}{}", batch_size, buffer);
            }
            else {
                log::info!("0");
            }
            buffer.clear();
        }

        else {
            LOGGER.clear_buffer();
            buffer.clear();
        }
    }
}