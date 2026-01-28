use embassy_time::Ticker;
use embassy_time::Duration;
use crate::resources::global_resources::EVENT;
use crate::resources::global_resources::EventList;

#[embassy_executor::task]
pub async fn event_handler_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        let received_event = EVENT.get_event_item().await;
        
        match received_event {
            EventList::MotorMoveDone(motor_id) => {
                log::info!("event MotorMoveDone {}", motor_id);    
            }
        }
        
        ticker.next().await;
    }
}