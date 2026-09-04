use super::*;

#[derive(Debug)]
struct MotorRoutineFinalizationError {
    messages: Vec<String>,
}

impl std::fmt::Display for MotorRoutineFinalizationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Motor routine finalization failed: {}",
            self.messages.join("; ")
        )
    }
}

impl std::error::Error for MotorRoutineFinalizationError {}

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

pub fn finalize_motor_routine(
    motor: &Arc<Mutex<Motor>>,
    logger: &Arc<Mutex<Logger>>,
    move_status: Result<(), Box<dyn std::error::Error>>,
) -> Result<(String, String), Box<dyn std::error::Error>> {
    let motor_stop_result = try_lock!(motor => stop_motor()).and_then(|result| result);

    let disable_result = if motor_stop_result.is_err() {
        Some(try_lock!(motor => disable()).and_then(|result| result))
    } else {
        None
    };

    let logger_stop_result = try_lock!(logger => stop()).and_then(|result| result);

    let mut errors = Vec::new();

    if let Err(error) = move_status {
        errors.push(format!("Movement Failed: {error}"));
    }

    if let Err(error) = motor_stop_result {
        errors.push(format!("Normal Motor Stop Failed: {error}"));
    }

    if let Some(Err(error)) = disable_result {
        errors.push(format!("Priority Motor Disable Failed: {error}"));
    }

    let log_paths = match logger_stop_result {
        Ok(paths) => Some(paths),
        Err(error) => {
            errors.push(format!("Logger Shutdown Failed: {error}"));
            None
        }
    };

    if errors.is_empty()
        && let Some(paths) = log_paths
    {
        return Ok(paths);
    }

    Err(Box::new(MotorRoutineFinalizationError { messages: errors }))
}

pub fn get_move_timeout_ms<'a>(
    initial_pos: &'a Position,
    target_pos: &'a Position,
    speed: &'a Speed,
    acc: &'a Acceleration,
    max_speed: i32,
) -> Result<u64, Box<dyn std::error::Error>> {
    let motion_profile = TrapezoidProfile::new(
        I32F32::from_num(initial_pos.count),
        I32F32::from_num(target_pos.count),
        I32F32::from_num(speed.cps.clamp(-max_speed, max_speed)),
        I32F32::from_num(acc.cps_square),
    )?;

    let timeout_ms: u64 = motion_profile
        .get_total_time_ms()
        .saturating_mul(TIMEOUT_SCALE)
        .saturating_add(TIMEOUT_OFFSET_MS)
        .max(DEFAULT_TIMEOUT_MS);

    Ok(timeout_ms)
}
