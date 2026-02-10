#![no_std]
#![no_main]

// Mod
mod tasks;
mod resources;
mod control;

// Resources
use crate::resources::gpio_list::Irqs;
use crate::resources::gpio_list::AssignedResources;
use crate::resources::gpio_list::MotorResources;
use crate::resources::gpio_list::EncoderResources;
use crate::resources::logger_resources::LoggerHandler;
use crate::resources::motor_resources::MotorHandler;
use crate::resources::MOVING_AVERAGE_WINDOW;
use crate::resources::USB_STATE;
use crate::resources::CONFIG_DESC;
use crate::resources::BOS_DESC;
use crate::resources::CONTROL_BUF;
use crate::resources::CORE1_STACK;
use crate::resources::EXECUTOR1;
use crate::resources::EXECUTOR0;
use crate::resources::EXECUTOR_HIGH;

// Tasks
use crate::tasks::usb_task::usb_device_task;
use crate::tasks::usb_task::usb_communication_task;
use crate::tasks::usb_task::usb_traffic_controller_task;
use crate::tasks::logger::firmware_logger_task;
use crate::tasks::dc_motor::motor_task;
use crate::tasks::encoder::encoder_task;

//  Struct
use crate::tasks::dc_motor::DCMotor;
use crate::tasks::encoder::RotaryEncoder;

// Control
use crate::control::MovingAverage;

// Library
use defmt_rtt as _;
use panic_probe as _;

use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;
use embassy_rp::Peri;
use embassy_rp::multicore::spawn_core1;
use embassy_rp::interrupt;
use embassy_rp::usb::Driver;
use embassy_rp::pio::Pio;
use embassy_rp::pio::PioPin;
use embassy_rp::pio::Instance;
use embassy_rp::pio::Common;
use embassy_rp::pio::StateMachine;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pio_programs::rotary_encoder::PioEncoderProgram;
use embassy_rp::pio_programs::pwm::PioPwmProgram;
use embassy_rp::pio_programs::pwm::PioPwm;
use embassy_executor::Executor;
use embassy_executor::Spawner;

struct DCMotorBuilder { }

impl DCMotorBuilder {

    fn build<'d, const SM0: usize, const SM1: usize, const SM2: usize, const N:usize, T:Instance, P0:PioPin, P1:PioPin, P2:PioPin, P3:PioPin> (
        pwm_cw_pin: Peri<'d, P0>, 
        pwm_ccw_pin: Peri<'d, P1>, 
        encoder_pin_a: Peri<'d, P2>, 
        encoder_pin_b: Peri<'d, P3>,
        filter: MovingAverage::<N>,
        motor_handler: &'static MotorHandler, 
        common: &mut Common<'d, T>, 
        sm0: StateMachine<'d, T, SM0>,
        sm1: StateMachine<'d, T, SM1>,
        sm2: StateMachine<'d, T, SM2>,

    ) -> (DCMotor<'d, T, SM1, SM2>, RotaryEncoder <'d, T, SM0, N>) {
        // Build DC Motor
        let pwm_prg = PioPwmProgram::new(common);
        let pwm_cw = PioPwm::new(common, sm1, pwm_cw_pin, &pwm_prg);
        let pwm_ccw = PioPwm::new(common, sm2, pwm_ccw_pin, &pwm_prg);
        let dc_motor = DCMotor::new(pwm_cw, pwm_ccw, &motor_handler);

        // Build Rotary Encoder
        let enc_prg = PioEncoderProgram::new(common);
        let pio_encoder = PioEncoder::new(common, sm0, encoder_pin_a, encoder_pin_b, &enc_prg);
        let encoder = RotaryEncoder::new(pio_encoder, &motor_handler, filter);
    
        (dc_motor, encoder)
    }
}

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

//  Create Motor and Logger
pub static MOTOR_0: MotorHandler = MotorHandler::new(0);
pub static LOGGER: LoggerHandler = LoggerHandler::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let ph = embassy_rp::init(Default::default());
    let p =  split_resources!(ph);

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

    let mut builder = {
        let builder = embassy_usb::Builder::new(
            usb_driver,
            usb_config,
            CONFIG_DESC.init([0; 256]),
            BOS_DESC.init([0; 256]),
            &mut [],
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    let class = CdcAcmClass::new(&mut builder, USB_STATE.init(State::new()), 64);
    let usb_dev = builder.build();

    let Pio {
        mut common, sm0, sm1, sm2, ..
    } = Pio::new(ph.PIO0, Irqs);

    let (dc_motor, encoder) = DCMotorBuilder::build(
        p.motor_resources.Motor0_PWM_CW_PIN, 
        p.motor_resources.Motor0_PWM_CCW_PIN,
        p.encoder_resources.Encoder0_PIN_A,
        p.encoder_resources.Encoder0_PIN_B,
        MovingAverage::<MOVING_AVERAGE_WINDOW>::new(),
        &MOTOR_0,
        &mut common, sm0, sm1, sm2
    );
    
    spawn_core1(
        ph.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.must_spawn(motor_task(dc_motor));
                spawner.must_spawn(encoder_task(encoder));  
            });
        },
    );   

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(usb_device_task(usb_dev));
        spawner.must_spawn(usb_communication_task(class));
        spawner.must_spawn(usb_traffic_controller_task());
        spawner.must_spawn(firmware_logger_task());
    });
}