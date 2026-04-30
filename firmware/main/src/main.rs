#![no_std]
#![no_main]

// Mod
mod control;
mod resources;
mod tasks;

use defmt_rtt as _;
use panic_probe as _;
use portable_atomic as _;

// Crate
use crate::resources::BOS_DESC;
use crate::resources::CONFIG_DESC;
use crate::resources::CONTROL_BUF;
use crate::resources::CORE1_STACK;
use crate::resources::EXECUTOR_HIGH;
use crate::resources::EXECUTOR0;
use crate::resources::EXECUTOR1;
use crate::resources::FLASH_SIZE;
use crate::resources::N_MOTOR;
use crate::resources::STORAGE;
use crate::resources::STORAGE_END;
use crate::resources::STORAGE_START;
use crate::resources::SYSTEM_FREQ_HZ;
use crate::resources::USB_STATE;
use crate::resources::gpio_list::AssignedResources;
use crate::resources::gpio_list::Irqs;
use crate::resources::gpio_list::Motor0Resources;
use crate::resources::gpio_list::Motor1Resources;
use crate::resources::logger_resources::LoggerHandler;
use crate::resources::motor_resources::MotorHandler;
use crate::tasks::dc_motor::DCMotor;
use crate::tasks::dc_motor::MotorPin;
use crate::tasks::dc_motor::motor0_task;
use crate::tasks::dc_motor::motor1_task;
use crate::tasks::heartbeat::heartbeat_task;
use crate::tasks::logger::firmware_logger_task;
use crate::tasks::usb_task::usb_device_task;
use crate::tasks::usb_task::usb_rx_task;
use crate::tasks::usb_task::usb_tx_task;

// Library
use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config;
use embassy_rp::flash::Async;
use embassy_rp::flash::Flash;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::interrupt;
use embassy_rp::multicore::spawn_core1;
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;
use sequential_storage::cache::NoCache;
use sequential_storage::map::MapConfig;
use sequential_storage::map::MapStorage;

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

//  Create Motor and Logger
pub static MOTOR: [MotorHandler; N_MOTOR] = create_motors!(0, 1);
pub static LOGGER: LoggerHandler = LoggerHandler::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Config::new(ClockConfig::system_freq(SYSTEM_FREQ_HZ).unwrap());

    let ph = embassy_rp::init(config);
    let p = split_resources!(ph);

    // On Board LED
    let mut onboard_led = ph.PIN_25;

    // FLash Storage
    let flash = Flash::<_, Async, FLASH_SIZE>::new(ph.FLASH, ph.DMA_CH0, Irqs);
    let storage_config = const { MapConfig::new(STORAGE_START..STORAGE_END) };
    let storage = MapStorage::<u8, _, _>::new(flash, storage_config, NoCache::new());

    {
        let mut guard = STORAGE.lock().await;
        *guard = Some(storage);
    }

    // USB Initialization
    let usb_driver = Driver::new(ph.USB, Irqs);

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
    // Split the class into transmitter and receiver
    let (usb_transmitter, usb_receiver) = class.split();

    // DC Motor Initialization
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(ph.PIO0, Irqs);

    // Build Motor 0
    let motor0_pin = MotorPin {
        pwm_cw_pin: p.motor_0.Motor_PWM_CW_PIN,
        pwm_ccw_pin: p.motor_0.Motor_PWM_CCW_PIN,
        pwm_slice: p.motor_0.SLICE,
        encoder_pin_a: p.motor_0.Encoder_PIN_A,
        encoder_pin_b: p.motor_0.Encoder_PIN_B,
    };

    let Some(motor0) = DCMotor::build(motor0_pin, &mut common, sm0, &MOTOR[0]).await else {
        {
            let mut led = Output::new(onboard_led.reborrow(), Level::Low);
            led.set_high();
        }
        return;
    };

    // Build Motor 1
    let motor1_pin = MotorPin {
        pwm_cw_pin: p.motor_1.Motor_PWM_CW_PIN,
        pwm_ccw_pin: p.motor_1.Motor_PWM_CCW_PIN,
        pwm_slice: p.motor_1.SLICE,
        encoder_pin_a: p.motor_1.Encoder_PIN_A,
        encoder_pin_b: p.motor_1.Encoder_PIN_B,
    };

    let Some(motor1) = DCMotor::build(motor1_pin, &mut common, sm1, &MOTOR[1]).await else {
        {
            let mut led = Output::new(onboard_led.reborrow(), Level::Low);
            led.set_high();
        }
        return;
    };

    // Blink after Initialization
    {
        let mut led = Output::new(onboard_led.reborrow(), Level::Low);
        for _ in 0..10 {
            led.toggle();
            Timer::after_millis(50).await;
        }
    }

    spawn_core1(
        ph.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(motor0_task(motor0).expect("FAILED"));
                spawner.spawn(motor1_task(motor1).expect("FAILED"));
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(usb_device_task(usb_dev).expect("FAILED"));
        spawner.spawn(usb_rx_task(usb_receiver).expect("FAILED"));
        spawner.spawn(usb_tx_task(usb_transmitter).expect("FAILED"));
        spawner.spawn(firmware_logger_task().expect("FAILED"));
        spawner.spawn(heartbeat_task(onboard_led.into()).expect("FAILED"));
    });
}
