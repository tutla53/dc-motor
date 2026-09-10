//! Iteration 1: reusable USB communication and generic command parsing.
//! One complete LED command is expected per received USB packet. Fragmented
//! or combined commands are rejected. Transport queues survive reconnects.
//! This application only controls the onboard LED, not the motor.
#![no_std]
#![no_main]

// Mod
mod resources;
mod tasks;

use defmt_rtt as _;
use panic_probe as _;
use portable_atomic as _;

// Crate
use crate::resources::BOS_DESC;
use crate::resources::CMD_CHANNEL;
use crate::resources::CONFIG_DESC;
use crate::resources::CONTROL_BUF;
use crate::resources::LED_STATUS;
use crate::resources::SYSTEM_FREQ_HZ;
use crate::resources::USB_BUFFER_SIZE;
use crate::resources::USB_RX_CHANNEL;
use crate::resources::USB_STATE;
use crate::tasks::led_task::led_task;
use crate::tasks::usb_task::usb_command_task;
use crate::tasks::usb_task::usb_device_task;
use crate::tasks::usb_task::usb_rx_task;
use crate::tasks::usb_task::usb_tx_task;

// Library
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::new(ClockConfig::system_freq(SYSTEM_FREQ_HZ).unwrap());
    let p = embassy_rp::init(config);

    /* --------------------------- Pin Assignment -------------------------- */
    let onboard_led = p.PIN_25;
    LED_STATUS.sender().send(false);

    /* --------------------------- Building USB Communication -------------------------- */
    let usb_driver = Driver::new(p.USB, Irqs);
    let usb_config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB_COMM_PLAYGROUND");
        // Separate from the motor host's 12345678 discovery prefix.
        config.serial_number = Some("USB-COMM-ITER1");
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
    let class = CdcAcmClass::new(
        &mut builder,
        USB_STATE.init(State::new()),
        USB_BUFFER_SIZE as u16,
    );
    let usb_dev = builder.build();
    let (usb_transmitter, usb_receiver) = class.split();

    /* --------------------------- Spawn Tasks -------------------------- */
    spawner.spawn(usb_device_task(usb_dev).expect("FAILED"));
    spawner.spawn(usb_rx_task(usb_receiver, USB_RX_CHANNEL.sender()).expect("FAILED"));
    spawner.spawn(usb_tx_task(usb_transmitter, CMD_CHANNEL.receiver()).expect("FAILED"));
    spawner
        .spawn(usb_command_task(USB_RX_CHANNEL.receiver(), CMD_CHANNEL.sender()).expect("FAILED"));
    spawner.spawn(led_task(onboard_led.into()).expect("FAILED"));
}
