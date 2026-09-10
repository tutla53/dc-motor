/*
* USB Task
    USB receiver
        ↓ run_rx()
    USB_RX_CHANNEL
        ↓ command task
    CommandHandler
        ↓
    CMD_CHANNEL ──┐
    EVENT_CHANNEL ├─ traffic-controller task → USB_TX_CHANNEL → run_tx()
    LOGGER queue ─┘

* USB commmand pattern
    Input Command Pattern
        [HEADER] [OP_CODE] [PARAMETERS]
    Output Pattern
        [HEADER] [ERROR_CODE] [OP_CODE] [DATA]
    Event Pattern
        [HEADER] [EVENT_CODE] [ID]
*/

use super::*;

/* --------------------------- USB Device -------------------------- */
#[embassy_executor::task]
pub async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

/* --------------------------- USB Receive -------------------------- */
#[embassy_executor::task]
pub async fn usb_rx_task(
    usb_receiver: Receiver<'static, Driver<'static, USB>>,
    packet_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, USB_RX_CHANNEL_SIZE>,
) {
    if run_rx(usb_receiver, packet_sender).await.is_err() {
        defmt::error!("USB RX stopped: buffer/configuration error");
    }
}

/* --------------------------- Command Processing -------------------------- */
#[embassy_executor::task]
pub async fn usb_command_task(
    packet_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, Packet, USB_RX_CHANNEL_SIZE>,
    command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
) {
    loop {
        let packet = packet_receiver.receive().await;
        let mut handler = CommandHandler::new(packet.as_slice(), command_sender);
        handler.process_command().await;
    }
}

/* --------------------------- USB Traffic Controller -------------------------- */
#[embassy_executor::task]
pub async fn usb_traffic_controller_task(
    command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
    event_receiver: ChannelReceiver<
        'static,
        CriticalSectionRawMutex,
        EventList,
        EVENT_CHANNEL_SIZE,
    >,
    log_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, LogData, LOG_BUFFER_SIZE>,
    packet_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, USB_TX_CHANNEL_SIZE>,
) {
    loop {
        let action = select3(
            command_receiver.receive(),
            event_receiver.receive(),
            log_receiver.receive(),
        )
        .await;

        let packet = match action {
            Either3::First(response) => {
                // COMMAND
                response
            }
            Either3::Second(event) => {
                // EVENT
                let mut buffer = Packet::new();
                buffer.push(UsbHeader::Event as u8).expect("Data Fits");

                match event {
                    EventList::MotorMoveDone(motor_id) => {
                        buffer
                            .push(0x00_u8)
                            .expect("Data Fits")
                            .push(motor_id)
                            .expect("Data Fits");
                    }
                }

                buffer
            }
            Either3::Third(log_data) => {
                // LOGGER
                if !LOGGER.is_logging_active() {
                    continue;
                }

                match log_data.pack_data() {
                    Ok(packet) => packet,
                    Err(_) => {
                        defmt::warn!("Logger packet encoding failed");
                        continue;
                    }
                }
            }
        };

        packet_sender.send(packet).await;
    }
}

/* --------------------------- USB Transmit -------------------------- */
#[embassy_executor::task]
pub async fn usb_tx_task(
    usb_transmitter: CdcAcmSender<'static, Driver<'static, USB>>,
    packet_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, Packet, USB_TX_CHANNEL_SIZE>,
) {
    if run_tx(usb_transmitter, packet_receiver).await.is_err() {
        defmt::error!("USB TX stopped: packet/buffer error");
    }
}
