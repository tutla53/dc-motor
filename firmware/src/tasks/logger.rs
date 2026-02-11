/*
* Logger Task
*/

use super::*;

use crate::MOTOR;
use crate::LOGGER;

// Resources
use crate::resources::logger_resources::LogData;

/* --------------------------- Code -------------------------- */
#[embassy_executor::task]
pub async fn firmware_logger_task() {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut start = Instant::now();
    let mut sequence: u8 = 0;

    loop {
        if LOGGER.is_logging_active() {
            if let Some(id) = LOGGER.get_motor_id() {
                let data = LogData {
                    seq: sequence,
                    dt: start.elapsed().as_millis() as u32,
                    values : [
                        MOTOR[id].get_current_pos(),
                        MOTOR[id].get_current_speed(),
                        MOTOR[id].get_commanded_pos(),
                        MOTOR[id].get_commanded_speed(),
                        MOTOR[id].get_commanded_pwm(),
                    ],
                };

                let _ = LOGGER.log_tx_buffer.try_send(data);
                sequence = sequence.wrapping_add(1);
            }

            ticker.next().await;
        }
        else {
            Timer::after(Duration::from_millis(100)).await;
            start = Instant::now();

            let new_time_sampling = LOGGER.get_logging_time_sampling();
            ticker = Ticker::every(Duration::from_millis(new_time_sampling));
            LOGGER.log_tx_buffer.clear();
            ticker.reset();
        }
    }
}