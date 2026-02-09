/* 
* Global Resources 
*/

/* --------------------------- Library -------------------------- */
use crate::MOTOR_0;
use crate::LOGGER;

use defmt_rtt as _;
use panic_probe as _;
use assign_resources::assign_resources;
use core::sync::atomic::AtomicI32;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use static_cell::StaticCell;

use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use embassy_rp::multicore::Stack;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_executor::Executor;
use embassy_executor::InterruptExecutor;
use embassy_usb::class::cdc_acm::State;

/* --------------------------- Declare Modules -------------------------- */
pub mod gpio_list;
pub mod motor_resources;
pub mod logger_resources;
pub mod event_resources;
pub mod usb_rx_resources;
pub mod usb_tx_resources;

pub use motor_resources::*;
pub use event_resources::*;
pub use usb_tx_resources::*;

/* --------------------------- USB Header -------------------------- */
pub const COMMAND_HEADER: u8 = 0xFF; 
pub const EVENT_HEADER: u8 = 0xFE;
pub const LOGGER_HEADER: u8 = 0xFD;

/* --------------------------- Sizes -------------------------- */
pub const USB_BUFFER_SIZE:  usize = 64;
pub const EVENT_CHANNEL_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;
pub const LOG_BUFFER_SIZE: usize = 256;
pub const MOVING_AVERAGE_WINDOW: usize = 10;

/* --------------------------- Channels-------------------------- */
pub static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, USB_BUFFER_SIZE> = Channel::new();
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, EventList, EVENT_CHANNEL_SIZE> = Channel::new();
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> = Channel::new();

/* --------------------------- Motor Tolerance-------------------------- */
pub const POS_TOLERANCE_COUNT: i32 = 5;
pub const SPEED_TOLERANCE_CPS: i32 = 2;
pub const SETTLE_TICKS: u32 = 20;

/* --------------------------- USB -------------------------- */
pub static USB_STATE: StaticCell<State> = StaticCell::new();
pub static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
pub static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
pub static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/* --------------------------- Executor -------------------------- */
pub static mut CORE1_STACK: Stack<4096> = Stack::new();
pub static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
pub static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();