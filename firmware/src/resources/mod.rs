/* 
* Global Resources 
*/

/* --------------------------- Library -------------------------- */
use crate::MOTOR_0;
use crate::MOTOR_1;
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
pub mod event_resources;
pub mod gpio_list;
pub mod logger_resources;
pub mod motor_resources;
pub mod usb_rx_resources;
pub mod usb_tx_resources;

pub use motor_resources::*;
pub use event_resources::*;
pub use usb_tx_resources::*;

/* --------------------------- USB Header -------------------------- */
#[derive(PartialEq, Clone)]
#[repr(u8)]
pub enum OpCode {
    None = 0,
    StartLogger = 1,
    StopLogger = 2,
    MoveMotorSpeed = 3,
    MoveMotorAbsPos = 4,
    StopMotor = 5,
    SetMotorPosPidParam = 6,
    GetMotorPosPidParam = 7,
    SetMotorSpeedPidParam = 8,
    GetMotorSpeedPidParam = 9,
    MoveMotorAbsPosTrapezoid = 10,
    GetMotorPos = 11,
    GetMotorSpeed = 12,
    MoveMotorOpenLoop = 13,
}

#[derive(PartialEq)]
#[repr(u8)]
pub enum HEADER {
    COMMAND = 0xFF,
    EVENT = 0xFE,
    LOGGER = 0xFD,
}

#[repr(u8)]
pub enum ErrorCode {
    NoError = 0,
    OpCodeNotFound = 1,
    NonFiniteFloat = 2,
    ReadByteError = 3,
    InvalidMotorId = 4,
    InvalidHeaderCode = 5,
    InvalidTimeSampling = 6,
}

/* --------------------------- Sizes -------------------------- */
pub const USB_BUFFER_SIZE:  usize = 64;
pub const EVENT_CHANNEL_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;
pub const LOG_BUFFER_SIZE: usize = 256;

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