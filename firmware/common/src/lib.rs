#![no_std]
use {defmt_rtt as _, panic_probe as _};

pub fn init() -> embassy_rp::Peripherals {
    embassy_rp::init(Default::default())
}
