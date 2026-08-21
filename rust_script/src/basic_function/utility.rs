use colored::Colorize;

use super::*;

pub fn wait_ms(duration_ms: u64) {
    thread::sleep(Duration::from_millis(duration_ms));
}

pub fn safe_exit(pico: Arc<Mutex<Pico>>, m0: Arc<Mutex<Motor>>, logger: Arc<Mutex<Logger>>) {
    println!("\n{}", "Perfroming Safe Exit!".bright_green().bold());

    let is_sim = try_lock!(pico => is_sim_mode()).unwrap_or(false);

    if !is_sim {
        println!("- Disabling Motor");
        let _ = try_lock!(m0 => disable());

        println!("- Stopping Logger");
        let _ = try_lock!(logger => stop());
    }
}
