//! Main
// DC Motor Ratio 1:4.4
// Encoder PPR = 11
// Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation

#![no_std]
#![no_main]

use {
    embassy_executor::Spawner,
    embassy_rp::bind_interrupts,
    embassy_rp::{
        peripherals::USB,
        peripherals::PIO0,
        usb::{Driver, InterruptHandler as UsbInterruptHandler},
        pio::{InterruptHandler, Pio},
        pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
    },
    embassy_time::{Ticker, Duration},
    defmt::*,
    core::sync::atomic::{AtomicI32, Ordering},
    {defmt_rtt as _, panic_probe as _},
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

static CURRENT_POS: AtomicI32 = AtomicI32::new(0);
static CURRENT_SPEED: AtomicI32 = AtomicI32::new(0); // count/s

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn encoder_0(mut encoder: PioEncoder<'static, PIO0, 0>) {
    let mut count = 0;
    loop {
        count += match encoder.read().await {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        CURRENT_POS.store(count, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn speed_0() {
    //count/s
    // 48.4  count/rotation
    // 1 count = 1/48.4 rotation

    let mut last_pos = 0;
    let dt = 50; 
    let mut ticker = Ticker::every(Duration::from_millis(dt));

    loop {
        let new_pos = CURRENT_POS.load(Ordering::Relaxed);
        let speed = (new_pos - last_pos) * (1000/dt) as i32;
        last_pos = new_pos;
        
        CURRENT_SPEED.store(speed, Ordering::Relaxed);

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner){
    let p = embassy_rp::init(Default::default());
    let usb_driver = Driver::new(p.USB, Irqs);

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let prg = PioEncoderProgram::new(&mut common);
    let encoder0 = PioEncoder::new(&mut common, sm0, p.PIN_4, p.PIN_5, &prg);
    
    unwrap!(spawner.spawn(logger_task(usb_driver)));
    spawner.must_spawn(encoder_0(encoder0));
    spawner.must_spawn(speed_0());
    
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        let rps =  CURRENT_SPEED.load(Ordering::Relaxed)as f32 /48.4;
        log::info!("Pos: {}, Speed: {:.2}", CURRENT_POS.load(Ordering::Relaxed), rps);
        ticker.next().await;
    }
}
