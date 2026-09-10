/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Sender as ChannelSender;
use embassy_sync::watch::Watch;
use embassy_usb::class::cdc_acm::State;
use static_cell::StaticCell;
use usb_comm::CommandReader;

/* --------------------------- Declare Modules -------------------------- */
pub mod config;
pub mod usb_resources;
pub mod usb_rx_resources;

pub use config::*;
pub use usb_resources::*;

pub type Packet = usb_comm::Packet<USB_BUFFER_SIZE>;
