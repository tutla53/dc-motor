#![no_std]
#![no_main]

// Mod
mod tasks;
mod resources;
mod control;

// Resources
use crate::resources::Irqs;
use crate::resources::AssignedResources;
use crate::resources::MotorResources;
use crate::resources::EncoderResources;
use crate::resources::LoggerHandler;
use crate::resources::MotorHandler;
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
use crate::tasks::dc_motor::DCMotor;
use crate::tasks::dc_motor::motor_task;
use crate::tasks::encoder::RotaryEncoder;
use crate::tasks::encoder::MovingAverage;
use crate::tasks::encoder::encoder_task;

// Library
use defmt_rtt as _;
use panic_probe as _;

use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;
use embassy_rp::multicore::spawn_core1;
use embassy_rp::interrupt;
use embassy_rp::usb::Driver;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pio_programs::rotary_encoder::PioEncoderProgram;
use embassy_rp::pio_programs::pwm::PioPwmProgram;
use embassy_rp::pio_programs::pwm::PioPwm;
use embassy_executor::Executor;
use embassy_executor::Spawner;

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

pub static MOTOR_0: MotorHandler = MotorHandler::new(0);
pub static LOGGER: LoggerHandler = LoggerHandler::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let ph = embassy_rp::init(Default::default());
    let p =  split_resources!(ph);
    let usb_driver = Driver::new(ph.USB, Irqs);

    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-serial example");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    let mut builder = {
        let builder = embassy_usb::Builder::new(
            usb_driver,
            config,
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

    let enc_prg = PioEncoderProgram::new(&mut common);
    let pwm_prg = PioPwmProgram::new(&mut common);
    
    let pwm_cw = PioPwm::new(
                    &mut common, 
                    sm1, 
                    p.motor_resources.Motor0_PWM_CW_PIN, 
                    &pwm_prg
                );
    let pwm_ccw = PioPwm::new(
                    &mut common, 
                    sm2, 
                    p.motor_resources.Motor0_PWM_CCW_PIN, 
                    &pwm_prg
                );
    let dc_motor = DCMotor::new(pwm_cw, pwm_ccw, &MOTOR_0);
    let filter = MovingAverage::<10>::new();
    let encoder = RotaryEncoder::new(PioEncoder::new(
                        &mut common, 
                        sm0, 
                        p.encoder_resources.Encoder0_PIN_A, 
                        p.encoder_resources.Encoder0_PIN_B, 
                        &enc_prg,
                    ),
                    &MOTOR_0,
                    filter,
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