/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::Packet;
use crate::communication::EventList;

use assign_resources::assign_resources;
use embassy_executor::Executor;
use embassy_executor::InterruptExecutor;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
use embassy_rp::multicore::Stack;
use embassy_rp::peripherals;
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::cdc_acm::State;
use fixed::types::I32F32;
use motor_control::PIDConfig;
use static_cell::StaticCell;

/* --------------------------- Declare Modules -------------------------- */
pub mod config;
pub mod gpio_list;

pub use config::*;

/* --------------------------- Version Parser -------------------------- */
const fn const_parse_u8(s: &str, component_idx: usize) -> u8 {
    let bytes = s.as_bytes();
    let mut current_idx = 0;
    let mut val = 0;
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'.' {
            if current_idx == component_idx {
                return val;
            }
            current_idx += 1;
            val = 0;
        } else if current_idx == component_idx {
            val = val * 10 + (bytes[i] - b'0');
        }
        i += 1;
    }
    val
}
