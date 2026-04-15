/*
* USB Task
*/

use super::*;

#[embassy_executor::task]
pub async fn usb_traffic_controller_task() {
    loop {
        let response = CMD_CHANNEL.receive().await;
        USB_TX_CHANNEL.send(response).await;
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
                Either::First(result) => match result {
                    Ok(len) => {
                        let data = &rx_buf[..len];
                        if !data.is_empty() {
                            let mut handler = CommandHandler::new(data);
                            handler.process_command().await;
                        }
                    }
                    Err(_) => break,
                },
                Either::Second(packet) => {
                    let _ = class.write_packet(packet.as_slice()).await;
                }
            }
        }
    }
}
