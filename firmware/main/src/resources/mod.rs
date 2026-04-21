/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::LOGGER;
use crate::MOTOR;
use assign_resources::assign_resources;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicI32;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use embassy_executor::Executor;
use embassy_executor::InterruptExecutor;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
use embassy_rp::flash::Async;
use embassy_rp::flash::Flash;
use embassy_rp::multicore::Stack;
use embassy_rp::peripherals;
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_usb::class::cdc_acm::State;
use sequential_storage::cache::NoCache;
use sequential_storage::map::MapStorage;
use serde::Deserialize;
use serde::Serialize;
use serde::de::DeserializeOwned;
use static_cell::StaticCell;

/* --------------------------- Declare Modules -------------------------- */
#[macro_use]
pub mod macros;

pub mod config;
pub mod flash_storage;
pub mod gpio_list;
pub mod logger_resources;
pub mod motor_resources;
pub mod usb_resources;
pub mod usb_rx_resources;
pub mod usb_tx_resources;

pub use config::*;
pub use flash_storage::*;
pub use motor_resources::*;
pub use usb_resources::*;
pub use usb_tx_resources::*;
