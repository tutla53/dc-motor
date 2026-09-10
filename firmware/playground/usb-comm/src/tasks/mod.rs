/*
* Tasks Hub
*/
use crate::resources::DATA_CHANNEL_SIZE;
use crate::resources::LED_STATUS;
use crate::resources::Packet;
use crate::resources::usb_rx_resources::CommandHandler;
use embassy_rp::Peri;
use embassy_rp::gpio::AnyPin;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver as ChannelReceiver;
use embassy_sync::channel::Sender as ChannelSender;
use embassy_usb::class::cdc_acm::Receiver;
use embassy_usb::class::cdc_acm::Sender as CdcAcmSender;
use usb_comm::transport::run_rx;
use usb_comm::transport::run_tx;

pub mod led_task;
pub mod usb_task;
