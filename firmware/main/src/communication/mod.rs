/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::LOGGER;
use crate::MOTOR;
use crate::Packet;
use crate::StoredMaxSpeed;
use crate::flash_storage::ConfigType;
use crate::flash_storage::save_config;
use crate::motor::MotorCommand;
use crate::motor::MotorHandler;
use crate::motor::Shape;
use crate::resources::DATA_CHANNEL_SIZE;
use crate::resources::FW_VERSION_MAJOR;
use crate::resources::FW_VERSION_MINOR;
use crate::resources::FW_VERSION_PATCH;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender as ChannelSender;
use fixed::types::I32F32;
use motor_control::PIDConfig;
use usb_comm::CommandError;
use usb_comm::CommandOpcode;
use usb_comm::CommandReader;
use usb_comm::create_opcode_enum;
use usb_comm::packet::PacketError;
use usb_comm::reader::ReadError;

/* --------------------------- Declare Modules -------------------------- */
pub mod api_config;
pub mod command_handler;
pub mod error_helper;

pub use api_config::*;
pub use command_handler::*;
