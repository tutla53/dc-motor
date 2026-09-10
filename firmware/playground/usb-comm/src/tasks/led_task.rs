/*
* LED Task
*/
use super::*;

#[embassy_executor::task]
pub async fn led_task(led_pin: Peri<'static, AnyPin>) {
    let mut led = Output::new(led_pin, Level::Low);
    let mut receiver = LED_STATUS.receiver().unwrap();
    loop {
        if receiver.changed().await {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}
