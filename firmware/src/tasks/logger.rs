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
enum LogMask {
    MotorPosition = 1 << 0,
    MotorSpeed = 1 << 1,
    CommandedPosition = 1 << 2,
    CommandedSpeed = 1 << 3,
    CommandedPwm = 1 << 4,
}

pub struct LogData {
    mask: u32,
    dt: u64,
    values: [i32; 5],
}

impl LogData {
    async fn select(mask: u32, dt: u64) -> Self {
        Self {
            mask,
            dt,
            values: [
                if (mask & LogMask::MotorPosition as u32) != 0 {MOTOR_0.get_current_pos().await} else {0},
                if (mask & LogMask::MotorSpeed as u32) != 0 {MOTOR_0.get_current_speed().await} else {0},
                if (mask & LogMask::CommandedPosition as u32) != 0 {MOTOR_0.get_commanded_pos().await} else {0},
                if (mask & LogMask::CommandedSpeed as u32) != 0 {MOTOR_0.get_commanded_speed().await} else {0},
                if (mask & LogMask::CommandedPwm as u32) != 0 {MOTOR_0.get_commanded_pwm().await} else {0},   
            ]         
        }
    }
    
    fn write_to_buffer(&self, buffer: &mut heapless::String<1024>) {
        write!(buffer, " {}", self.dt).unwrap();

        for i in 0..5 {
            if (self.mask & (1 << i)) != 0 {
                write!(buffer, " {}", self.values[i]).unwrap();
            }
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
            let mask = LOGGER.get_log_mask().await;
            let dt = start.elapsed().as_millis();
            
            let data = LogData::select(mask, dt).await;

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