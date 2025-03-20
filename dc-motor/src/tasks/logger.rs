/*
* Logger Task
*/

use {
    crate::resources::{
        global_resources:: {
            MOTOR_0,
            LOGGER,
        },
    },
    embassy_sync::{
        channel::Channel,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    core::fmt::Write,
    embassy_time::{Ticker, Duration, Instant, Timer},
    {defmt_rtt as _, panic_probe as _},
};

static LOGGER_CONTROL: Channel<CriticalSectionRawMutex, LogData, 2048> = Channel::new();

fn send_logged_data(data: LogData) {
    let _ = LOGGER_CONTROL.try_send(data);
}

async fn get_logged_data() -> LogData {
    return LOGGER_CONTROL.receive().await;
}

fn get_logged_data_len() -> usize {
    return LOGGER_CONTROL.len();
}

fn clear_logged_data() {
    LOGGER_CONTROL.clear();
}

enum LogMask {
    MotorPosition = 1 << 0,
    MotorSpeed = 1 << 1,
    CommandedPosition = 1 << 2,
    CommandedSpeed = 1 << 3,
}

struct LogData {
    mask: u32,
    dt: u64,
    motor_speed: i32,
    motor_position: i32,
    commanded_speed: i32,
    commanded_position: i32,
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
        }
    }
    
    fn process(self) ->  heapless::String::<128> {
        let mut output = heapless::String::<128>::new();

        write!(&mut output, "{}", self.dt).unwrap();

        if (self.mask & LogMask::MotorSpeed as u32) != 0 {
            write!(&mut output, " {}", self.motor_speed).unwrap();
        }
        if (self.mask & LogMask::MotorPosition as u32) != 0 {
            write!(&mut output, " {}", self.motor_position).unwrap();
        }
        if (self.mask & LogMask::CommandedSpeed as u32) != 0 {
            write!(&mut output, " {}", self.commanded_speed).unwrap();
        }
        if (self.mask & LogMask::CommandedPosition as u32) != 0 {
            write!(&mut output, " {}", self.commanded_position).unwrap();
        }

        return output;
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

            send_logged_data(data);
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
        let command = LOGGER.get_logged_item().await;

        if command {
            let len = get_logged_data_len();
            let count = len.min(50);

            if count > 0 {
                for _ in 0..count {
                    let data = get_logged_data().await;   
                    write!(&mut output, " {}", data.process()).unwrap(); 
                }

                log::info!("{}{}", count, output);
            }
            else {
                log::info!("0");
            }
            output.clear();
        }

        else {
            clear_logged_data();
            output.clear();
        }
    }
}