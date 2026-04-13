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
use crate::resources::ConfigType;
use crate::resources::EXECUTOR_HIGH;
use crate::resources::EXECUTOR0;
use crate::resources::EXECUTOR1;
use crate::resources::FLASH_SIZE;
use crate::resources::N_MOTOR;
use crate::resources::PWM_PERIOD_TICKS;
use crate::resources::STORAGE;
use crate::resources::STORAGE_END;
use crate::resources::STORAGE_START;
use crate::resources::SYSTEM_FREQ_HZ;
use crate::resources::USB_STATE;
use crate::resources::gpio_list::AssignedResources;
use crate::resources::gpio_list::Irqs;
use crate::resources::gpio_list::Motor0Resources;
use crate::resources::gpio_list::Motor1Resources;
use crate::resources::load_config;
use crate::resources::logger_resources::LoggerHandler;
use crate::resources::motor_resources::MotorHandler;
use crate::tasks::dc_motor::DCMotor;
use crate::tasks::dc_motor::motor0_task;
use crate::tasks::dc_motor::motor1_task;
use crate::tasks::heartbeat::heartbeat_task;
use crate::tasks::logger::firmware_logger_task;
use crate::tasks::usb_task::usb_communication_task;
use crate::tasks::usb_task::usb_device_task;
use crate::tasks::usb_task::usb_traffic_controller_task;

// Library
use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config;
use embassy_rp::flash::Async;
use embassy_rp::flash::Flash;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::interrupt;
use embassy_rp::multicore::spawn_core1;
use embassy_rp::pio::Common;
use embassy_rp::pio::Instance;
use embassy_rp::pio::Pio;
use embassy_rp::pio::PioPin;
use embassy_rp::pio::StateMachine;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pio_programs::rotary_encoder::PioEncoderProgram;
use embassy_rp::pwm::ChannelAPin;
use embassy_rp::pwm::ChannelBPin;
use embassy_rp::pwm::Config as PwmConfig;
use embassy_rp::pwm::Pwm;
use embassy_rp::pwm::Slice;
use embassy_rp::usb::Driver;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;
use sequential_storage::cache::NoCache;
use sequential_storage::map::MapConfig;
use sequential_storage::map::MapStorage;

struct MotorPin<'d, S: Slice, P0: ChannelBPin<S>, P1: ChannelAPin<S>, P2: PioPin, P3: PioPin> {
    pwm_cw_pin: Peri<'d, P0>,
    pwm_ccw_pin: Peri<'d, P1>,
    pwm_slice: Peri<'d, S>,
    encoder_pin_a: Peri<'d, P2>,
    encoder_pin_b: Peri<'d, P3>,
}

struct DCMotorBuilder {}

impl DCMotorBuilder {
    async fn build<
        'd,
        T: Instance,
        S: Slice,
        const SM: usize,
        P0: ChannelBPin<S>,
        P1: ChannelAPin<S>,
        P2: PioPin,
        P3: PioPin,
    >(
        motor_pin: MotorPin<'d, S, P0, P1, P2, P3>,
        common: &mut Common<'d, T>,
        sm: StateMachine<'d, T, SM>,
        motor_handler: &'static MotorHandler,
    ) -> Option<DCMotor<'d, T, SM>> {
        // Build PWM
        let mut pwm_config = PwmConfig::default();
        pwm_config.top = PWM_PERIOD_TICKS; // 25kHz frequency @ 125MHz clock
        pwm_config.compare_a = 0;
        pwm_config.compare_b = 0;

        let pwm = Pwm::new_output_ab(
            motor_pin.pwm_slice,
            motor_pin.pwm_ccw_pin,
            motor_pin.pwm_cw_pin,
            pwm_config,
        );
        let (pwm_a, pwm_b) = pwm.split();

        let pwm_ccw = pwm_a?;
        let pwm_cw = pwm_b?;

        // Build Rotary Encoder
        let enc_prg = PioEncoderProgram::new(common);
        let pio_encoder = PioEncoder::new(
            common,
            sm,
            motor_pin.encoder_pin_a,
            motor_pin.encoder_pin_b,
            &enc_prg,
        );

        let dc_motor = DCMotor::new(pwm_cw, pwm_ccw, pio_encoder, motor_handler);

        // Initialized Motor Config
        let motor_id = motor_handler.id;

        let speed_pid = load_config(
            motor_id,
            ConfigType::SpeedPID,
            motor_handler.default_speed_pid,
        )
        .await;
        motor_handler.set_speed_pid(speed_pid).await;

        let pos_pid = load_config(
            motor_id,
            ConfigType::PositionPID,
            motor_handler.default_pos_pid,
        )
        .await;
        motor_handler.set_pos_pid(pos_pid).await;

        Some(dc_motor)
    }
}

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
    let flash = Flash::<_, Async, FLASH_SIZE>::new(ph.FLASH, ph.DMA_CH0);
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

    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(ph.PIO0, Irqs);

    let motor0_pin = MotorPin {
        pwm_cw_pin: p.motor_0.Motor_PWM_CW_PIN,
        pwm_ccw_pin: p.motor_0.Motor_PWM_CCW_PIN,
        pwm_slice: p.motor_0.SLICE,
        encoder_pin_a: p.motor_0.Encoder_PIN_A,
        encoder_pin_b: p.motor_0.Encoder_PIN_B,
    };

    let Some(motor0) = DCMotorBuilder::build(motor0_pin, &mut common, sm0, &MOTOR[0]).await else {
        {
            let mut led = Output::new(onboard_led.reborrow(), Level::Low);
            led.set_high();
        }
        return;
    };

    let motor1_pin = MotorPin {
        pwm_cw_pin: p.motor_1.Motor_PWM_CW_PIN,
        pwm_ccw_pin: p.motor_1.Motor_PWM_CCW_PIN,
        pwm_slice: p.motor_1.SLICE,
        encoder_pin_a: p.motor_1.Encoder_PIN_A,
        encoder_pin_b: p.motor_1.Encoder_PIN_B,
    };

    let Some(motor1) = DCMotorBuilder::build(motor1_pin, &mut common, sm1, &MOTOR[1]).await else {
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
                spawner.must_spawn(motor0_task(motor0));
                spawner.must_spawn(motor1_task(motor1));
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(usb_device_task(usb_dev));
        spawner.must_spawn(usb_communication_task(class));
        spawner.must_spawn(usb_traffic_controller_task());
        spawner.must_spawn(firmware_logger_task());
        spawner.must_spawn(heartbeat_task(onboard_led.into()));
    });
}
