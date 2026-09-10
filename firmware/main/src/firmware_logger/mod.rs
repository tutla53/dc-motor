/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::Packet;
use crate::communication::UsbHeader;
use crate::resources::LOG_BUFFER_SIZE;
use crate::resources::LOG_PACKET_SIZE;

use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use usb_comm::packet::PacketError;

/* --------------------------- Declare Modules -------------------------- */
pub mod logger_handler;

pub use logger_handler::*;
