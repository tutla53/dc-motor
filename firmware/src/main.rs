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
use crate::tasks::event_handler::event_handler_task;

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

use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/* --------------------------- Code -------------------------- */
pub struct Packet {
    pub data: [u8; 64],
    pub len: usize,
}

impl Packet {
    pub fn new() -> Self {
        Self { data: [0u8; 64], len: 0 }
    }

    pub fn from_str(s: &str) -> Self {
        let mut data = [0u8; 64];
        let bytes = s.as_bytes();
        let len = bytes.len().min(64);
        data[..len].copy_from_slice(&bytes[..len]);
        Self { data, len }
    }
    pub fn push_bytes(&mut self, bytes: &[u8]) {
        let remain = 64 - self.len;
        let to_copy = bytes.len().min(remain);
        self.data[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
    }
}

static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, 16> = Channel::new();

// 2. Static memory for USB (Required)
static STATE: StaticCell<State> = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::task]
async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn usb_communication_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    let mut rx_buf = [0u8; 64];
    let subscriber = USB_TX_CHANNEL.receiver();

    loop {
        class.wait_connection().await;
        
        loop {
            use embassy_futures::select::{select, Either};

            match select(class.read_packet(&mut rx_buf), subscriber.receive()).await {
                Either::First(result) => {
                    match result {
                        Ok(len) => {
                            if let Ok(data) = core::str::from_utf8(&rx_buf[..len]) {
                                let parts: Vec<&str, 8> = data.split_whitespace().collect();
                                if !parts.is_empty() {
                                    let handler = CommandHandler::new(&parts);
                                    handler.process_command().await;
                                }
                            }
                        }
                        Err(_e) => break,
                    }
                }
                Either::Second(packet) => {
                    let _ = class.write_packet(&packet.data[..packet.len]).await;
                }
            }
        }
    }
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

    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    
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

        spawner.must_spawn(firmware_logger_task());
        spawner.must_spawn(send_logger_task());
        spawner.must_spawn(event_handler_task());
    });
}