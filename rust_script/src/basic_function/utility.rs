
use super::*;

pub fn wait_ms(duration_ms: u64) {
    thread::sleep(Duration::from_millis(duration_ms));
}