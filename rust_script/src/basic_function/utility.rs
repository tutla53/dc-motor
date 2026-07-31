use colored::Colorize;

use super::*;

pub fn wait_ms(duration_ms: u64) {
    thread::sleep(Duration::from_millis(duration_ms));
}

pub fn safe_exit(pico: Arc<Mutex<Pico>>, m0: Arc<Mutex<Motor>>, logger: Arc<Mutex<Logger>>) {
    println!("\n{}", "Perfroming Safe Exit!".bright_green().bold());

    let is_sim = run_with_lock!(pico => is_sim_mode()).unwrap_or(false);

    if !is_sim {
        println!("- Stopping Motor");
        let _ = run_with_lock!(m0 => stop_motor());

        println!("- Stopping Logger");
        let _ = run_with_lock!(logger => stop());
    }
}
