/*
* Tasks Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::LOGGER;
use crate::MOTOR;
use crate::Packet;
use crate::StoredMaxSpeed;
use crate::communication::CommandHandler;
use crate::communication::EventList;
use crate::communication::UsbHeader;
use crate::firmware_logger::logger_handler::LogData;
use crate::flash_storage::ConfigType;
use crate::flash_storage::load_config;
use crate::flash_storage::save_config;
use crate::motor::motor_handler::ControlMode;
use crate::motor::motor_handler::MotorCommand;
use crate::motor::motor_handler::MotorHandler;
use crate::motor::motor_handler::Shape;
use crate::resources::DATA_CHANNEL_SIZE;
use crate::resources::DEFAULT_MOTOR_CONTROL_MAX_SPEED_PPS;
use crate::resources::DEFAULT_PID_POS_CONFIG;
use crate::resources::DEFAULT_PID_SPEED_CONFIG;
use crate::resources::EVENT_CHANNEL_SIZE;
use crate::resources::LOG_BUFFER_SIZE;
use crate::resources::PHYSICAL_MOTOR_MAX_SPEED_PPS;
use crate::resources::POS_TOLERANCE_PULSE;
use crate::resources::PWM_PERIOD_TICKS;
use crate::resources::SETTLE_TICKS;
use crate::resources::SPEED_FILTER_WINDOW;
use crate::resources::SPEED_TOLERANCE_PPS;
use crate::resources::TICKS_TO_PPS_PER_WINDOWS;
use crate::resources::TIME_SAMPLING_S_FIXED;
use crate::resources::TIME_SAMPLING_US;
use crate::resources::USB_RX_CHANNEL_SIZE;
use crate::resources::USB_TX_CHANNEL_SIZE;

use embassy_futures::select::Either3;
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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver as ChannelReceiver;
use embassy_sync::channel::Sender as ChannelSender;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Ticker;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::Receiver;
use embassy_usb::class::cdc_acm::Sender as CdcAcmSender;
use fixed::types::I16F16;
use fixed::types::I32F32;
use motor_control::MovingAverageFilter;
use motor_control::PIDController;
use motor_control::TrapezoidProfile;
use usb_comm::transport::run_rx;
use usb_comm::transport::run_tx;

pub mod dc_motor;
pub mod heartbeat;
pub mod logger;
pub mod usb_task;
