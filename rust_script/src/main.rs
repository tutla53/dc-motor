mod board;
mod basic_function;
mod logger;
mod config;

use crate::board::rpi::Pico;
use crate::basic_function::move_motor::Motor;
use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Speed;
use crate::basic_function::conversion::Acceleration;
use crate::logger::fwlogger::Logger;
use crate::basic_function::utility::wait_ms;

use std::time::Duration;
use std::sync::{Arc, Mutex};

include!(concat!(env!("OUT_DIR"), "/generated_commands.rs"));

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (pico, log_rx) = Pico::new(env!("CONFIG_FILE"))?;
    let shared_pico = Arc::new(Mutex::new(pico));
    
    let motor_id = 0;
    let m0 = Motor::new(Arc::clone(&shared_pico), motor_id);
    let mut logger = Logger::new(Arc::clone(&shared_pico), log_rx, m0.motor_id);

    let current_pos = m0.get_motor_pos()?;
    println!("Initial Pos: {} count, {:.2} rotation", current_pos.count, current_pos.rotation);

    let target_rotation = if current_pos.rotation != 0.0 { 0.0 } else { 20.0 };
    let log_mask = 5;
    let time_sampling = 1;

    m0.clear_motor_event();

    logger.start(log_mask, time_sampling);
    wait_ms(300);

    m0.move_motor_pos_trapezoid(
        Position::from_rotation(target_rotation), 
        Speed::from_rpm(1000.0), 
        Acceleration::from_cps_sq(10_000),
    );
    
    m0.wait_move_done(Duration::from_secs(20))?;
    
    wait_ms(300);
    logger.stop();
    m0.stop_motor();

    let current_pos = m0.get_motor_pos()?;
    println!("Final Pos: {} count, {:.2} rotation", current_pos.count, current_pos.rotation);
    
    Ok(())
}