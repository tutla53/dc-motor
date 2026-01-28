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
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Ticker;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Timer;

/* --------------------------- Code -------------------------- */
static LOGGER_BUFFER: Channel<CriticalSectionRawMutex, LogData, 2048> = Channel::new();

fn send_data_to_buffer(data: LogData) {
    let _ = LOGGER_BUFFER.try_send(data);
}

fn clear_buffer() {
    LOGGER_BUFFER.clear();
}

enum LogMask {
    MotorPosition = 1 << 0,
    MotorSpeed = 1 << 1,
    CommandedPosition = 1 << 2,
    CommandedSpeed = 1 << 3,
    CommandedPwm = 1 << 4,
}

struct LogData {
    mask: u32,
    dt: u64,
    motor_speed: i32,
    motor_position: i32,
    commanded_speed: i32,
    commanded_position: i32,
    commanded_pwm: i32,
}

impl LogData {
    async fn select(mask: u32, dt: u64) -> Self {
        Self {
            mask,
            dt,
            motor_speed: if (mask & LogMask::MotorSpeed as u32) != 0 {
                MOTOR_0.get_current_speed().await
            } else {
                0
            },
            motor_position: if (mask & LogMask::MotorPosition as u32) != 0 {
                MOTOR_0.get_current_pos().await
            } else {
                0
            },
            commanded_speed: if (mask & LogMask::CommandedSpeed as u32) != 0 {
                MOTOR_0.get_commanded_speed().await
            } else {
                0
            },
            commanded_position: if (mask & LogMask::CommandedPosition as u32) != 0 {
                MOTOR_0.get_commanded_pos().await
            } else {
                0
            },
            commanded_pwm: if (mask & LogMask::CommandedPwm as u32) != 0 {
                MOTOR_0.get_commanded_pwm().await
            } else {
                0
            },            
        }
    }
    
    fn process(&self, output: &mut heapless::String<1024>) {
        // let mut output = heapless::String::<256>::new();

        write!(output, " {}", self.dt).unwrap();

        if (self.mask & LogMask::MotorSpeed as u32) != 0 {
            write!(output, " {}", self.motor_speed).unwrap();
        }
        if (self.mask & LogMask::MotorPosition as u32) != 0 {
            write!(output, " {}", self.motor_position).unwrap();
        }
        if (self.mask & LogMask::CommandedSpeed as u32) != 0 {
            write!(output, " {}", self.commanded_speed).unwrap();
        }
        if (self.mask & LogMask::CommandedPosition as u32) != 0 {
            write!(output, " {}", self.commanded_position).unwrap();
        }
        if (self.mask & LogMask::CommandedPwm as u32) != 0 {
            write!(output, " {}", self.commanded_pwm).unwrap();
        }

        // return output;
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
            
            // TODO: Optimize thise method --> reduce write! call
            let data = LogData::select(mask, dt).await;

            send_data_to_buffer(data);
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
    let mut output = heapless::String::<1024>::new();

    loop {
        let command = LOGGER.wait_for_log_request().await;

        if command {
            let count = LOGGER_BUFFER.len().min(50);

            if count > 0 {
                for _ in 0..count {
                    if let Ok(data) = LOGGER_BUFFER.try_receive() {   
                        data.process(&mut output);
                        // write!(&mut output, " {}", data.process()).unwrap(); 
                    }
                }

                log::info!("log {}{}", count, output);
            }
            else {
                log::info!("0");
            }
            output.clear();
        }

        else {
            clear_buffer();
            output.clear();
        }
    }
}