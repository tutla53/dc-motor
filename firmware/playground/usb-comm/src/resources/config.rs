/*
* Configuration Resources - Iteration 1
*/
use super::*;

/* --------------------------- System Clock -------------------------- */
pub const SYSTEM_FREQ_HZ: u32 = 133_000_000;

/* --------------------------- USB Communication -------------------------- */
pub const USB_BUFFER_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;

/* --------------------------- Communication Channels -------------------------- */
pub static USB_RX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> =
    Channel::new();
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> =
    Channel::new();

/* --------------------------- USB Builder -------------------------- */
pub static USB_STATE: StaticCell<State> = StaticCell::new();
pub static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
pub static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
pub static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/* --------------------------- LED Resources -------------------------- */
pub const ONBOARD_LED_ID: i32 = 25;
// Reports the requested state; the GPIO task applies it asynchronously.
pub static LED_STATUS: Watch<CriticalSectionRawMutex, bool, 4> = Watch::new();
