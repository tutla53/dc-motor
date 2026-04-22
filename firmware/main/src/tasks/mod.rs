/*
* Tasks Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::LOGGER;
use crate::MOTOR;
use crate::control::PIDcontrol;
use crate::control::TrapezoidProfile;
use crate::resources::CMD_CHANNEL;
use crate::resources::ConfigType;
use crate::resources::DEFAULT_PID_POS_CONFIG;
use crate::resources::DEFAULT_PID_SPEED_CONFIG;
use crate::resources::EVENT_CHANNEL;
use crate::resources::MOTOR_MAX_SPEED_CPS;
use crate::resources::POS_TOLERANCE_COUNT;
use crate::resources::PWM_PERIOD_TICKS;
use crate::resources::SETTLE_TICKS;
use crate::resources::SPEED_FILTER_WINDOW;
use crate::resources::SPEED_TOLERANCE_CPS;
use crate::resources::TICKS_TO_CPS;
use crate::resources::TIME_SAMPLING_US;
use crate::resources::USB_BUFFER_SIZE;
use crate::resources::USB_TX_CHANNEL;
use crate::resources::UsbHeader;
use crate::resources::load_config;
use crate::resources::logger_resources::LogData;
use crate::resources::motor_resources::ControlMode;
use crate::resources::motor_resources::MotorCommand;
use crate::resources::motor_resources::MotorHandler;
use crate::resources::motor_resources::Shape;
use crate::resources::usb_resources::EventList;
use crate::resources::usb_rx_resources::CommandHandler;
use crate::resources::usb_tx_resources::Packet;

use embassy_futures::select::Either;
use embassy_futures::select::Either3;
use embassy_futures::select::select;
use embassy_futures::select::select3;
use embassy_rp::Peri;
use embassy_rp::gpio::AnyPin;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::USB;
use embassy_rp::pio::Common;
use embassy_rp::pio::Instance;
use embassy_rp::pio::PioPin;
use embassy_rp::pio::StateMachine;
use embassy_rp::pio_programs::rotary_encoder::Direction;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pio_programs::rotary_encoder::PioEncoderProgram;
use embassy_rp::pwm::ChannelAPin;
use embassy_rp::pwm::ChannelBPin;
use embassy_rp::pwm::Config as PwmConfig;
use embassy_rp::pwm::Pwm;
use embassy_rp::pwm::PwmOutput;
use embassy_rp::pwm::SetDutyCycle;
use embassy_rp::pwm::Slice;
use embassy_rp::usb::Driver;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Ticker;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use fixed::types::I16F16;
use fixed::types::I32F32;

pub mod dc_motor;
pub mod heartbeat;
pub mod logger;
pub mod usb_task;
