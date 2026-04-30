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
pub async fn usb_tx_task(
    mut usb_transmitter: CdcAcmSender<'static, Driver<'static, USB>>,
    command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
    event_receiver: ChannelReceiver<
        'static,
        CriticalSectionRawMutex,
        EventList,
        EVENT_CHANNEL_SIZE,
    >,
    log_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, LogData, LOG_BUFFER_SIZE>,
) {
    let mut packet = Packet::new();

    loop {
        usb_transmitter.wait_connection().await;

        loop {
            let action = select3(
                command_receiver.receive(), // Priority 1
                event_receiver.receive(),   // Priority 2
                log_receiver.receive(),     // Priority 3
            )
            .await;

            packet.clear();

            match action {
                Either3::First(response) => {
                    // COMMAND
                    packet = response;
                }

                Either3::Second(event) => {
                    // EVENT
                    packet.data[0] = UsbHeader::Event as u8; // Event Header

                    match event {
                        EventList::MotorMoveDone(motor_id) => {
                            packet.data[1] = 0x00;
                            packet.data[2] = motor_id;
                        }
                    }

                    packet.len = 3;
                }

                Either3::Third(data_1) => {
                    // LOGGER
                    if LOGGER.is_logging_active() {
                        data_1.pack_data(&mut packet.data[0..26]);
                        packet.len = 26;
                    } else {
                        continue;
                    }
                }
            }

            // Send the Packet
            if usb_transmitter
                .write_packet(packet.as_slice())
                .await
                .is_err()
            {
                break;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn usb_rx_task(
    mut usb_receiver: Receiver<'static, Driver<'static, USB>>,
    command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
) {
    let mut rx_buf = [0u8; USB_BUFFER_SIZE];

    loop {
        usb_receiver.wait_connection().await;

        loop {
            match usb_receiver.read_packet(&mut rx_buf).await {
                Ok(len) if len > 0 => {
                    let mut handler = CommandHandler::new(&rx_buf[..len], command_sender);
                    handler.process_command().await;
                }
                Ok(_) => continue,
                Err(_) => break,
            }
        }
    }
}
