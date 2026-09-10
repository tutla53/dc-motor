/*
* USB Tasks
*/
use super::*;

#[embassy_executor::task]
pub async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
pub async fn usb_rx_task(
    usb_receiver: Receiver<'static, Driver<'static, USB>>,
    packet_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
) {
    if run_rx(usb_receiver, packet_sender).await.is_err() {
        defmt::error!("USB RX stopped: buffer/configuration error");
    }
}

#[embassy_executor::task]
pub async fn usb_tx_task(
    usb_transmitter: CdcAcmSender<'static, Driver<'static, USB>>,
    command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
) {
    if run_tx(usb_transmitter, command_receiver).await.is_err() {
        defmt::error!("USB TX stopped: packet/buffer error");
    }
}

#[embassy_executor::task]
pub async fn usb_command_task(
    packet_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
    command_sender: ChannelSender<'static, CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE>,
) {
    loop {
        let packet = packet_receiver.receive().await;
        let mut handler = CommandHandler::new(packet.as_slice(), command_sender);
        handler.process_command().await;
    }
}
