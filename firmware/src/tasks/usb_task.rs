/*
* USB Task
*/

// Resources
use crate::resources::global_resources::LOGGER;
use crate::resources::global_resources::CMD_CHANNEL;
use crate::resources::global_resources::EVENT_CHANNEL;
use crate::resources::global_resources::USB_TX_CHANNEL;
use crate::resources::global_resources::Packet;
use crate::resources::global_resources::USB_BUFFER_SIZE;
use crate::resources::global_resources::EventList;
use crate::tasks::usb_handler::CommandHandler;

// Library
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_futures::select::select;
use embassy_futures::select::select3;
use embassy_futures::select::Either;
use embassy_futures::select::Either3;

#[embassy_executor::task]
pub async fn usb_traffic_controller_task() {

    loop {
        let action = select3(
            CMD_CHANNEL.receive(),          // Priority 1
            EVENT_CHANNEL.receive(),        // Priority 2
            LOGGER.log_tx_buffer.receive(), // Priority 3
        ).await;

        let mut p = Packet::new();

        match action {
            Either3::First(packet) => {
                let _ = USB_TX_CHANNEL.send(packet).await;
            }

            Either3::Second(event) => {
                p.data[0] = 0xCC; // Event Header
                
                match event {
                    EventList::MotorMoveDone(motor_id) => {
                        p.data[0] = motor_id;
                    }
                }

                p.len = 8;
                let _ = USB_TX_CHANNEL.send(p).await;
            }

            Either3::Third(data_1) => {
                if LOGGER.is_logging_active().await {
                    data_1.pack_data(&mut p.data[0..32]);
                    p.len = 32;

                    if let Ok(data_2) = LOGGER.log_tx_buffer.try_receive() {
                        data_2.pack_data(&mut p.data[32..64]);
                        p.len = 64;
                    }
                    let _ = USB_TX_CHANNEL.send(p).await;
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
pub async fn usb_communication_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    let mut rx_buf = [0u8; USB_BUFFER_SIZE];
    let subscriber = USB_TX_CHANNEL.receiver();

    loop {
        class.wait_connection().await;
        
        loop {
            match select(class.read_packet(&mut rx_buf), subscriber.receive()).await {
                Either::First(result) => {
                    match result {
                        Ok(len) => {
                            let data = &rx_buf[..len];
                            if !data.is_empty() {
                                let handler = CommandHandler::new(data);
                                handler.process_command().await;
                            }
                        }
                        Err(_) => break,
                    }
                }
                Either::Second(packet) => {
                    let _ = class.write_packet(&packet.data[..packet.len]).await;
                }
            }
        }
    }
}