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
    embassy_time::{Ticker, Duration, Instant, with_timeout},
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
    const COUNT_THRESHOLD: i32 = 3;
    const TIMEOUT: u64 = 100;

    let mut count = 0;
    let mut delta_count: i32 = 0;
    let mut start = Instant::now();

    loop {
        match with_timeout(Duration::from_millis(TIMEOUT), encoder.read()).await {
            Ok(value) => {
                match value {
                    Direction::Clockwise => {
                        count += 1;
                        delta_count += 1;
                    },
                    Direction::CounterClockwise =>{
                        count -= 1;
                        delta_count -= 1;
                    },
                }
            },
            Err (_) => {},
        };

        let dt = start.elapsed().as_millis();
        if (delta_count.abs() >= COUNT_THRESHOLD) || (dt > TIMEOUT) {
            let speed = delta_count  * (1000 / dt as i32);
            delta_count = 0;
            start = Instant::now();
            CURRENT_SPEED.store(speed, Ordering::Relaxed);
        }

        CURRENT_POS.store(count, Ordering::Relaxed);
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
    // spawner.must_spawn(speed_0());
    
    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        let rps =  CURRENT_SPEED.load(Ordering::Relaxed)as f32 /48.4;
        log::info!("1500, -1500, {:.2}", rps*60.0);
        ticker.next().await;
    }
}
