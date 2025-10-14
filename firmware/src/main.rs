#![no_std]
#![no_main]

// Mod
mod tasks;
mod resources;

// Resources
use crate::resources::global_resources::MOTOR_0;
use crate::resources::gpio_list::Irqs;
use crate::resources::gpio_list::AssignedResources;
use crate::resources::gpio_list::MotorResources;
use crate::resources::gpio_list::EncoderResources;

// Tasks
use crate::tasks::usb_handler::CommandHandler;
use crate::tasks::logger::firmware_logger_task;
use crate::tasks::logger::send_logger_task;
use crate::tasks::dc_motor::DCMotor;
use crate::tasks::dc_motor::motor_task;
use crate::tasks::encoder::RotaryEncoder;
use crate::tasks::encoder::MovingAverage;
use crate::tasks::encoder::encoder_task;

// Library
use core::str;
use heapless::Vec;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pio_programs::rotary_encoder::PioEncoderProgram;
use embassy_rp::pio_programs::pwm::PioPwmProgram;
use embassy_rp::pio_programs::pwm::PioPwm;
use embassy_usb_logger::ReceiverHandler;

/* --------------------------- Code -------------------------- */
struct UsbHandler;

impl ReceiverHandler for UsbHandler {
    async fn handle_data(&self, raw_data: &[u8]) {
        if let Ok(raw_data) = str::from_utf8(raw_data) {

            let parts: Vec<&str, 8> = raw_data.split_whitespace().collect();

            if !parts.is_empty() { 
                let handler = CommandHandler::new(&parts);
                handler.process_command().await;
            }
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, UsbHandler);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let ph = embassy_rp::init(Default::default());
    let p =  split_resources!(ph);
    let usb_driver = Driver::new(ph.USB, Irqs);

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
    
    spawner.must_spawn(usb_logger_task(usb_driver));
    spawner.must_spawn(encoder_task(encoder));
    spawner.must_spawn(motor_task(dc_motor));
    spawner.must_spawn(firmware_logger_task());
    spawner.must_spawn(send_logger_task());

}