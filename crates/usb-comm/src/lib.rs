//! USB communication, iteration 1.
//!
//! RX items are USB packets, not assembled application commands. TX retains
//! queued packets across reconnects and does not retry a failed in-flight item.
#![no_std]

pub mod command;
mod macros;
pub mod packet;
pub mod reader;
pub mod transport;

pub use command::{CommandError, CommandOpcode, CommandReader};
pub use packet::Packet;
pub use reader::ByteReader;

#[cfg(test)]
mod tests;
