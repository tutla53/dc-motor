/*
    Task
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::resources::TIME_SAMPLING_US;
use crate::resources::TICKS_TO_CPS;
use crate::resources::DEFAULT_PID_POS_CONFIG;
use crate::resources::DEFAULT_PID_SPEED_CONFIG;

use embassy_futures::select::select;
use embassy_futures::select::select3;
use embassy_futures::select::Either;
use embassy_futures::select::Either3;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::Instance;
use embassy_rp::pio_programs::rotary_encoder::Direction;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pwm::PwmOutput;
use embassy_rp::pwm::SetDutyCycle;
use embassy_time::Timer;
use embassy_time::Ticker;
use embassy_time::Instant;
use embassy_time::Duration;
use embassy_usb::class::cdc_acm::CdcAcmClass;

use fixed::types::I16F16;
use fixed::types::I32F32;

pub mod logger;
pub mod dc_motor;
pub mod usb_task;