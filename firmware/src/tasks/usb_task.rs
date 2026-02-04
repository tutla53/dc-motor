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
use crate::resources::global_resources::EVENT_HEADER;
use crate::tasks::usb_handler::CommandHandler;

// Library
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_futures::select::select;
use embassy_futures::select::select3;
use embassy_futures::select::Either;
use embassy_futures::select::Either3;

/*
    Prototype

    Header
    - Command Header 0xFF (255)
    - Event Header 0xFE (254)
    - Logger Header 0XFD (253)

    Event Message Format
    - [header opcode id]
    - e.g [0xFE (Event) 0x00 (MotorMoveDone) 0x00 (motor_id)]

    Command Message Format
    - [header opcode [data]]
    - e.g. [0xFF (Command) 0x08 (set_motor_speed_pid_param) [kp, ki, kd] (data)]

*/

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
                let _ = USB_TX_CHANNEL.try_send(packet);
            }

            Either3::Second(event) => {
                p.data[0] = EVENT_HEADER; // Event Header
                
                match event {
                    EventList::MotorMoveDone(motor_id) => {
                        p.data[1] = 0x00;
                        p.data[2] = motor_id;
                    }
                }

                p.len = 3;
                let _ = USB_TX_CHANNEL.try_send(p);
            }

            Either3::Third(data_1) => {
                if LOGGER.is_logging_active() {
                    data_1.pack_data(&mut p.data[0..26]);
                    p.len = 26;
                    let _ = USB_TX_CHANNEL.try_send(p);
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
                                let mut handler = CommandHandler::new(data);
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