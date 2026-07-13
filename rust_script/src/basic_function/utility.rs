
use super::*;

#[allow(unused)]
pub fn wait_ms(duration_ms: u64) {
    thread::sleep(Duration::from_millis(duration_ms));
}

pub fn safe_exit() {
    println!("Perfroming Safe Exit!");
    let shared = SHARED.get().expect("Shared resources not initialized!");

    if let Ok(m0) = shared.m0.lock() {
        m0.stop_motor();
    }

    if let Ok(mut logger) = shared.logger.lock() {
        logger.stop();
    }
}