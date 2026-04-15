// Simple USB Communication with Raw Byte Method via USB Serial CDCACM with Logger
// Reference: https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_serial_with_logger.rs

#![no_std]
#![no_main]

mod macros;
mod usb_rx_handler;
mod usb_task;
mod usb_tx_handler;

use defmt_rtt as _;
use panic_probe as _;
use portable_atomic as _;

use crate::usb_rx_handler::CommandHandler;
use crate::usb_task::usb_communication_task;
use crate::usb_task::usb_device_task;
use crate::usb_task::usb_traffic_controller_task;
use crate::usb_tx_handler::Packet;

use embassy_executor::Spawner;
use embassy_futures::select::Either;
use embassy_futures::select::select;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config;
use embassy_rp::gpio::AnyPin;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;
use static_cell::StaticCell;

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});

/* --------------------------- System Clock -------------------------- */
const SYSTEM_FREQ_HZ: u32 = 133_000_000; // 133 MHz

/* --------------------------- USB Builder -------------------------- */
static USB_STATE: StaticCell<State> = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/* --------------------------- Communication Channels -------------------------- */
const USB_BUFFER_SIZE: usize = 64;
const DATA_CHANNEL_SIZE: usize = 64;
static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, USB_BUFFER_SIZE> = Channel::new();
static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> = Channel::new();

/* --------------------------- Communication Architecture -------------------------- */
/*
    Command Format                  --> [HEADER: u8]    [OP_CODE:u8]    [PARAMETER]
        1. Turn on Onboard LED      --> [0xFF]          [0x01]          [LED_ID: i32]
        2. Turn off Onboard LED     --> [0xFF]          [0x02]          [LED_ID: i32]
        3. Get Onboard LED state    --> [0xFF]          [0x03]          [LED_ID: i32]

    Response:
        Format --> [HEADER: u8] [ERROR_CODE: u8] [OP_CODE:u8] [RESPONSE]

    Valid Onboard LED = 0x19 --> PIN_25
*/

#[derive(PartialEq)]
#[repr(u8)]
enum UsbHeader {
    Command = 0xFF,
}

create_opcode_enum! {
    #[derive(PartialEq, Clone, Copy, Debug)]
    #[repr(u8)]
    enum OpCode {
        None = 0,
        TurnOnLed = 1,
        TurnOffLed = 2,
        GetLedStatus = 3,
    }
}

#[repr(u8)]
enum ErrorCode {
    // Communication Error Code
    NoError = 0,
    OpCodeNotFound = 1,
    ReadByteError = 2,
    InvalidHeaderCode = 3,
    // Command Error Code
    InvalidLedId = 4,
}

/* --------------------------- LED Signal -------------------------- */
static LED_STATUS: Watch<CriticalSectionRawMutex, bool, 4> = Watch::new();
const ONBOARD_LED_ID: i32 = 25;

/* --------------------------- Task -------------------------- */
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::new(ClockConfig::system_freq(SYSTEM_FREQ_HZ).unwrap());
    let p = embassy_rp::init(config);

    let onboard_led = p.PIN_25;

    let usb_driver = Driver::new(p.USB, Irqs);

    let usb_config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB_DC_MOTOR");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        usb_config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, USB_STATE.init(State::new()), 64);
    let usb_dev = builder.build();

    spawner.must_spawn(usb_device_task(usb_dev));
    spawner.must_spawn(usb_communication_task(class));
    spawner.must_spawn(usb_traffic_controller_task());
    spawner.must_spawn(led_task(onboard_led.into()));
}

#[embassy_executor::task]
pub async fn led_task(led_pin: Peri<'static, AnyPin>) {
    let mut led = Output::new(led_pin, Level::Low);
    let mut receiver = LED_STATUS.receiver().unwrap();
    let snd = LED_STATUS.sender();

    snd.send(false);

    loop {
        let new_state = receiver.changed().await;

        if new_state {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}
