#![no_std]
#![no_main]

mod tasks;
mod resources;

use {
    crate::resources::{
        global_resources::MOTOR_0,
        gpio_list::{
            Irqs,
            AssignedResources,
            MotorResources,
            EncoderResources,
        },
    },
    crate::tasks::{
        usb_handler::CommandHandler,
        logger::{
            firmware_logger_task,
            send_logger_task,
        },
        dc_motor::{
            RotaryEncoder,
            DCMotor,
            encoder_task,
            motor_task,
        },
    },    
    embassy_executor::Spawner,
    embassy_rp::{
        peripherals::USB,
        usb::Driver,
        pio::Pio,
        pio_programs::{
            rotary_encoder::{PioEncoder, PioEncoderProgram},
            pwm::{PioPwmProgram, PioPwm},
        },
    },
    embassy_usb_logger::ReceiverHandler,
    core::str,
    heapless::Vec,
    {defmt_rtt as _, panic_probe as _},
};

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
    let encoder = RotaryEncoder::new(PioEncoder::new(
                        &mut common, 
                        sm0, 
                        p.encoder_resources.Encoder0_PIN_A, 
                        p.encoder_resources.Encoder0_PIN_B, 
                        &enc_prg
                    ),
                    &MOTOR_0
                );
    let dc_motor = DCMotor::new(pwm_cw, pwm_ccw, &MOTOR_0);

    spawner.must_spawn(usb_logger_task(usb_driver));
    spawner.must_spawn(encoder_task(encoder));
    spawner.must_spawn(motor_task(dc_motor));
    spawner.must_spawn(firmware_logger_task());
    spawner.must_spawn(send_logger_task());

}