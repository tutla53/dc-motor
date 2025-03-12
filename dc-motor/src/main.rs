#![no_std]
#![no_main]

mod tasks;
mod resources;

use {
    crate::resources::{
        gpio_list::Irqs,
        global_resources::MotorId,
    },
    crate::tasks::{
        usb_handler::{
            handle_move_motor,
            handle_firmware_logger,
            handle_motor_pid,
        },
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
    embassy_time::Timer,
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
                match parts[0] {
                    "motor" => { handle_move_motor(&parts).await; },
                    "log"  => { handle_firmware_logger(&parts).await; },
                    "motor_pid" => { handle_motor_pid(&parts).await; },
                    _ => { log::info!("Command not found"); },
                }
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
    let p = embassy_rp::init(Default::default());
    let usb_driver = Driver::new(p.USB, Irqs);

    let Pio {
        mut common, sm0, sm1, sm2, ..
    } = Pio::new(p.PIO0, Irqs);

    let enc_prg = PioEncoderProgram::new(&mut common);
    let pwm_prg = PioPwmProgram::new(&mut common);
    
    let pwm_cw = PioPwm::new(&mut common, sm1, p.PIN_15, &pwm_prg);
    let pwm_ccw = PioPwm::new(&mut common, sm2, p.PIN_14, &pwm_prg);
    let dc_motor = DCMotor::new(pwm_cw, pwm_ccw, MotorId::Motor0);
    let encoder = RotaryEncoder::new(PioEncoder::new(&mut common, sm0, p.PIN_6, p.PIN_7, &enc_prg), MotorId::Motor0);

    spawner.must_spawn(usb_logger_task(usb_driver));
    spawner.must_spawn(encoder_task(encoder));
    spawner.must_spawn(motor_task(dc_motor));
    spawner.must_spawn(firmware_logger_task());
    spawner.must_spawn(send_logger_task());

    // loop {
    //     Timer::after_millis(100).await;
    // }

}