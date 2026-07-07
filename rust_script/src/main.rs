mod board;
mod basic_function;
mod logger;

use crate::board::rpi::Pico;
use crate::basic_function::move_motor::Motor;
use crate::logger::fwlogger::Logger;

use std::thread;
use std::time::Duration;
use std::sync::{Arc, Mutex};

include!(concat!(env!("OUT_DIR"), "/generated_commands.rs"));

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (pico, log_rx) = Pico::new(env!("CONFIG_FILE"))?;
    let shared_pico = Arc::new(Mutex::new(pico));

    if let Ok(mut pico) = shared_pico.lock() {
        match pico.get_firmware_version() {
            Ok((major, minor, patch)) => println!("Firmware: {}.{}.{}", major, minor, patch),
            Err(e) => println!("Error: {}", e),
        }
    }
    
    let m0 = Motor::new(Arc::clone(&shared_pico), 0);
    let mut logger = Logger::new(Arc::clone(&shared_pico), log_rx);

    m0.clear_motor_event();
    logger.start(5, 1);
    thread::sleep(Duration::from_millis(500));

    m0.move_motor_trapezoid(10_000.0, 1000.0, 10_000.0);
    m0.wait_move_done(Duration::from_secs(20))?;

    thread::sleep(Duration::from_millis(500));
    logger.stop();
    
    if let Ok(mut pico) = shared_pico.lock() {
        match pico.get_motor_pos(0) {
            Ok(value) => println!("Final Motor Pos = {}", value),
            Err(e) => println!("Error: {}", e),
        }
    }

    Ok(())
}