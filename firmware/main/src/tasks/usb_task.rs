/*
* USB Task
*/

use super::*;

/*
    Input Command Pattern
        [HEADER] [OP_CODE] [PARAMETERS]
    Output Pattern
        [HEADER] [ERROR_CODE] [OP_CODE] [DATA]
    Event Pattern
        [HEADER] [EVENT_CODE] [ID]
*/

#[embassy_executor::task]
pub async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
pub async fn usb_tx_task(mut usb_transmitter: Sender<'static, Driver<'static, USB>>) {
    loop {
        usb_transmitter.wait_connection().await;

        loop {
            let action = select3(
                CMD_CHANNEL.receive(),          // Priority 1
                EVENT_CHANNEL.receive(),        // Priority 2
                LOGGER.log_tx_buffer.receive(), // Priority 3
            )
            .await;

            let mut p = Packet::new();

            match action {
                Either3::First(response) => {
                    // COMMAND
                    p = response;
                }

                Either3::Second(event) => {
                    // EVENT
                    p.data[0] = UsbHeader::Event as u8; // Event Header

                    match event {
                        EventList::MotorMoveDone(motor_id) => {
                            p.data[1] = 0x00;
                            p.data[2] = motor_id;
                        }
                    }

                    p.len = 3;
                }

                Either3::Third(data_1) => {
                    // LOGGER
                    if LOGGER.is_logging_active() {
                        data_1.pack_data(&mut p.data[0..26]);
                        p.len = 26;
                    } else {
                        continue;
                    }
                }
            }

            // Send the Packet
            if usb_transmitter.write_packet(p.as_slice()).await.is_err() {
                break;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn usb_rx_task(mut usb_receiver: Receiver<'static, Driver<'static, USB>>) {
    let mut rx_buf = [0u8; USB_BUFFER_SIZE];

    loop {
        usb_receiver.wait_connection().await;

        loop {
            match usb_receiver.read_packet(&mut rx_buf).await {
                Ok(len) if len > 0 => {
                    let mut handler = CommandHandler::new(&rx_buf[..len]);
                    handler.process_command().await;
                }
                Ok(_) => continue,
                Err(_) => break,
            }
        }
    }
}
