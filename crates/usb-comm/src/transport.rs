/*
* USB Transport - Iteration 1
*/
use crate::packet::Packet;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Receiver as ChannelReceiver;
use embassy_sync::channel::Sender as ChannelSender;
use embassy_usb::class::cdc_acm::Receiver as UsbReceiver;
use embassy_usb::class::cdc_acm::Sender as UsbSender;
use embassy_usb::driver::Driver;
use embassy_usb::driver::EndpointError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    RxBufferTooSmall,
    TxPacketTooLarge,
    BufferOverflow,
}

/// Queue USB packets, without application command reassembly.
/// A full channel applies backpressure. Queued RX data survives reconnects.
pub async fn run_rx<'d, 'ch, D, M, const N: usize, const Q: usize>(
    mut receiver: UsbReceiver<'d, D>,
    incoming: ChannelSender<'ch, M, Packet<N>, Q>,
) -> Result<(), TransportError>
where
    D: Driver<'d>,
    M: RawMutex,
{
    if N < usize::from(receiver.max_packet_size()) {
        return Err(TransportError::RxBufferTooSmall);
    }
    let mut buffer = [0u8; N];
    loop {
        receiver.wait_connection().await;
        loop {
            match receiver.read_packet(&mut buffer).await {
                Ok(0) => {}
                Ok(len) => {
                    let packet = Packet::from_slice(&buffer[..len])
                        .map_err(|_| TransportError::BufferOverflow)?;
                    incoming.send(packet).await;
                }
                Err(EndpointError::Disabled) => break,
                Err(EndpointError::BufferOverflow) => return Err(TransportError::BufferOverflow),
            }
        }
    }
}

/// Send one nonempty application packet per USB data packet.
/// Failed in-flight data is not retried; queued data survives reconnects.
/// Full-size writes are terminated with a zero-length USB packet.
pub async fn run_tx<'d, 'ch, D, M, const N: usize, const Q: usize>(
    mut sender: UsbSender<'d, D>,
    outgoing: ChannelReceiver<'ch, M, Packet<N>, Q>,
) -> Result<(), TransportError>
where
    D: Driver<'d>,
    M: RawMutex,
{
    let max_packet_size = usize::from(sender.max_packet_size());
    loop {
        sender.wait_connection().await;
        loop {
            let packet = outgoing.receive().await;
            if packet.is_empty() {
                continue;
            }
            if packet.len() > max_packet_size {
                return Err(TransportError::TxPacketTooLarge);
            }
            let result = async {
                sender.write_packet(packet.as_slice()).await?;
                if packet.len() == max_packet_size {
                    sender.write_packet(&[]).await?;
                }
                Ok::<(), EndpointError>(())
            }
            .await;
            match result {
                Ok(()) => {}
                Err(EndpointError::Disabled) => break,
                Err(EndpointError::BufferOverflow) => return Err(TransportError::BufferOverflow),
            }
        }
    }
}
