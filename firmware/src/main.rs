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
use static_cell::StaticCell;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Executor;
use embassy_executor::InterruptExecutor;
use embassy_rp::multicore::Stack;
use embassy_rp::multicore::spawn_core1;
use embassy_rp::interrupt;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::PioEncoder;
use embassy_rp::pio_programs::rotary_encoder::PioEncoderProgram;
use embassy_rp::pio_programs::pwm::PioPwmProgram;
use embassy_rp::pio_programs::pwm::PioPwm;
use embassy_usb_logger::ReceiverHandler;
use embassy_time::Duration;
use embassy_time::Ticker;

/* --------------------------- Code -------------------------- */
static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

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
async fn event_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut count = 0;

    loop {
        log::info!("event {}", count);
        count += 1;
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, UsbHandler);
}

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[cortex_m_rt::entry]
fn main() -> ! {
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
    
    spawn_core1(
        ph.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.must_spawn(motor_task(dc_motor))
            });
        },
    );   

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(encoder_task(encoder));  
        spawner.must_spawn(usb_logger_task(usb_driver));
        spawner.must_spawn(firmware_logger_task());
        spawner.must_spawn(send_logger_task());
    });
}