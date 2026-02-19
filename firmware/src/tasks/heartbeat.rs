/*
* Heartbeat Task
*/

use super::*;

#[embassy_executor::task]
pub async fn heartbeat_task(led_pin: Peri<'static, AnyPin>) {
    let mut led = Output::new(led_pin, Level::Low);
    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}