
use super::*;

pub fn wait_ms(duration_ms: u64) {
    thread::sleep(Duration::from_millis(duration_ms));
}

pub fn safe_exit(
    pico: Arc<Mutex<Pico>>, 
    m0: Arc<Mutex<Motor>>, 
    logger: Arc<Mutex<Logger>>
) {
    println!("Perfroming Safe Exit!");
    
    let is_sim = {
        if let Ok(pico) = pico.lock() { pico.sim_mode } 
        else { false }
    };

    if !is_sim {
        if let Ok(m0) = m0.lock() {
            m0.stop_motor();
        }

        if let Ok(mut logger) = logger.lock() {
            logger.stop();
        }
    }
}