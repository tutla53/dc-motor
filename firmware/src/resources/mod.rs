/* 
* Global Resources 
*/

/* --------------------------- Library -------------------------- */
use crate::MOTOR;
use crate::LOGGER;

use defmt_rtt as _;
use panic_probe as _;
use assign_resources::assign_resources;
use core::sync::atomic::AtomicU8;
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
pub mod usb_resources;
pub mod gpio_list;
pub mod logger_resources;
pub mod motor_resources;
pub mod usb_rx_resources;
pub mod usb_tx_resources;
pub mod macros;

pub use motor_resources::*;
pub use usb_resources::*;
pub use usb_tx_resources::*;

/* --------------------------- Motor PID Config-------------------------- */
pub const DEFAULT_PID_POS_CONFIG: PIDConfig = PIDConfig {
    kp: 25.0,
    ki: 0.0,
    kd: 5.0,
    i_limit: 1500.0,
};

pub const DEFAULT_PID_SPEED_CONFIG: PIDConfig = PIDConfig {
    kp: 2.0,
    ki: 0.8,
    kd: 10.0,
    i_limit: 4999.0,
};

/* --------------------------- USB -------------------------- */
pub const USB_BUFFER_SIZE:  usize = 64;
pub const EVENT_CHANNEL_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;
pub const LOG_BUFFER_SIZE: usize = 256;
pub const N_MOTOR: usize = 2;

/* --------------------------- Channels-------------------------- */
pub static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, USB_BUFFER_SIZE> = Channel::new();
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, EventList, EVENT_CHANNEL_SIZE> = Channel::new();
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> = Channel::new();

/* --------------------------- Motor-------------------------- */
pub const POS_TOLERANCE_COUNT: i32 = 5;
pub const SPEED_TOLERANCE_CPS: i32 = 2;
pub const SETTLE_TICKS: u32 = 20;
pub const PWM_PERIOD_TICKS: u16 = 4999;
pub const MOTOR_MAX_PWM_TICKS: i32 = 4999; // Calibrated Value
pub const MOTOR_MAX_SPEED_CPS: i32 = 1130; // 1400 RPM
pub const TIME_SAMPLING_US: u64 = 5000;
pub const TICKS_TO_CPS: f32 = 1_000_000.0_f32 / TIME_SAMPLING_US as f32;
pub const SPEED_FILTER_WINDOW: usize = 1 << 3; // Must be 2^n

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