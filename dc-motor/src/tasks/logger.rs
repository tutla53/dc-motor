/*
* Logger Task
*/

use {
    crate::resources::{
        global_resources as global,
        global_resources::{
            LogMask,
            CommandedSpeed,
            MotorId::Motor0,
        },
    },
    embassy_time::{Ticker, Duration, Instant, Timer},
    {defmt_rtt as _, panic_probe as _},
};

#[embassy_executor::task]
pub async fn firmware_logger_task() {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut start = Instant::now();

    loop {
        let logging = global::is_logging_active().await;

        if logging {
            let data = LogMask{
                dt: start.elapsed().as_millis(),
                motor_speed: global::get_current_speed(Motor0).await
            };
            global::send_logged_data(data).await;
            ticker.next().await;
        }
        else {
            Timer::after(Duration::from_millis(200)).await;
            start = Instant::now();
            let new_time_sampling = global::get_logging_time_sampling().await;
            ticker = Ticker::every(Duration::from_millis(new_time_sampling));
            ticker.reset();
        }
    }
}

#[embassy_executor::task]
pub async fn send_logger_task() {
    loop {
        let command = global::get_logged_data().await;
        let commanded_speed = global::get_commanded_speed(Motor0).await;
        match commanded_speed {
            CommandedSpeed::Speed(value) => {
                log::info!("{} {} {}", command.dt, command.motor_speed, value);
            }
            CommandedSpeed::Stop => {
                log::info!("{} {} {}", command.dt, command.motor_speed, 0);
            },
        }
    }
}