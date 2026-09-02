use super::*;

// TODO: Add the guidline to create the custom script

pub fn enable_m0() -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let is_enabled = try_lock!(shared.m0 => is_enabled())??;

    if !is_enabled {
        println!("  [INFO] - Enabling motor...");
        try_lock!(shared.m0 => enable())??;
        println!("  [DONE] - Motor has been enabled");

        return Ok(());
    } else {
        println!("Motor is already enabled");
    }

    Ok(())
}

pub fn open_loop(pwm: i32) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    /* ---------- Config ---------- */
    let log_mask = LogMask::CommandedPwm | LogMask::MotorSpeed;
    let time_sampling = 1;
    let chart_title = "Open Loop Response";
    let y_label = "PWM (ticks), Velocity (RPM)";
    let duration_ms = 1500;

    /* ---------- Move Motor ---------- */
    try_lock!(shared.m0 => clear_motor_event())?;
    try_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    let move_status = (|| -> Result<(), Box<dyn std::error::Error>> {
        try_lock!(shared.m0 => move_motor_open_loop(pwm))??;
        wait_ms(duration_ms);
        Ok(())
    })();

    let motor_stop_result = try_lock!(shared.m0 => stop_motor());
    let logger_stop_result = try_lock!(shared.logger => stop());

    move_status?;
    motor_stop_result??;

    /* ---------- Plot Firmware Log ---------- */
    let (log_dir, file_dir) = logger_stop_result??;

    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_open_loop(&csv_log)?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}

pub fn pos_trapezoid_move(
    target_rotation: f64,
    speed_rpm: f64,
    acc_cps2: i32,
) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    /* ---------- Config ---------- */
    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;
    let chart_title = "Trapezoid Position Control";
    let y_label = "Position (rotation)";

    /* ---------- Gathering Motor Info ---------- */
    let initial_pos = try_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        initial_pos.count, initial_pos.rotation
    );
    let pid_speed_config = try_lock!(shared.m0 => get_pid_motor_speed())??;
    let pid_pos_config = try_lock!(shared.m0 => get_pid_motor_pos())??;
    let max_speed_pps = try_lock!(shared.m0 => get_motor_max_speed())??;

    /* ---------- Move Motor ---------- */
    try_lock!(shared.m0 => clear_motor_event())?;
    try_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    let move_status = (|| -> Result<(), Box<dyn std::error::Error>> {
        try_lock!(
            shared.m0 =>
            move_motor_pos_trapezoid(
                Position::from_rotation(target_rotation),
                Speed::from_rpm(speed_rpm),
                Acceleration::from_cps_sq(acc_cps2)
            )
        )??;

        if let Err(e) = try_lock!(shared.m0 => wait_move_done(Duration::from_secs(20)))? {
            println!("{}", e);
        }
        wait_ms(300);

        Ok(())
    })();

    let motor_stop_result = try_lock!(shared.m0 => stop_motor());
    let logger_stop_result = try_lock!(shared.logger => stop());

    move_status?;
    motor_stop_result??;

    /* ---------- Get Motor Pos ---------- */
    let current_pos = try_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    /* ---------- Plot Firmware Log ---------- */
    let (log_dir, file_dir) = logger_stop_result??;

    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_position_control(
        &csv_log,
        initial_pos.count,
        max_speed_pps,
        &pid_speed_config,
        &pid_pos_config,
    )?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}

pub fn pos_step_move(target_rotation: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    /* ---------- Config ---------- */
    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;
    let chart_title = "Step Position Response";
    let y_label = "Position (rotation)";

    /* ---------- Gathering Motor Info ---------- */
    let initial_pos = try_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        initial_pos.count, initial_pos.rotation
    );
    let pid_speed_config = try_lock!(shared.m0 => get_pid_motor_speed())??;
    let pid_pos_config = try_lock!(shared.m0 => get_pid_motor_pos())??;
    let max_speed_pps = try_lock!(shared.m0 => get_motor_max_speed())??;

    /* ---------- Move Motor ---------- */
    try_lock!(shared.m0 => clear_motor_event())?;
    try_lock!(shared.logger=> start(log_mask, time_sampling))??;
    wait_ms(300);

    let move_status = (|| -> Result<(), Box<dyn std::error::Error>> {
        try_lock!(
            shared.m0 => move_motor_pos_step(Position::from_rotation(target_rotation))
        )??;

        if let Err(e) = try_lock!(shared.m0 => wait_move_done(Duration::from_secs(20)))? {
            println!("{}", e);
        }
        wait_ms(300);

        Ok(())
    })();

    let motor_stop_result = try_lock!(shared.m0 => stop_motor());
    let logger_stop_result = try_lock!(shared.logger => stop());

    move_status?;
    motor_stop_result??;

    /* ---------- Get Motor Pos ---------- */
    let current_pos = try_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    /* ---------- Plot Firmware Log ---------- */
    let (log_dir, file_dir) = logger_stop_result??;

    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_position_control(
        &csv_log,
        initial_pos.count,
        max_speed_pps,
        &pid_speed_config,
        &pid_pos_config,
    )?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}

pub fn speed_move(target_speed: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    /* ---------- Config ---------- */
    let log_mask = LogMask::CommandedSpeed | LogMask::MotorSpeed;
    let time_sampling = 1;
    let chart_title = "Closed Loop Velocity Response";
    let y_label = "Velocity (RPM)";
    let duration_ms = 1500;

    /* ---------- Gathering Motor Info ---------- */
    let pid_config = try_lock!(shared.m0 => get_pid_motor_speed())??;
    let max_speed_pps = try_lock!(shared.m0 => get_motor_max_speed())??;

    /* ---------- Move Motor ---------- */
    try_lock!(shared.m0 => clear_motor_event())?;
    try_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    let move_status = (|| -> Result<(), Box<dyn std::error::Error>> {
        try_lock!(shared.m0 => move_motor_speed(Speed::from_rpm(target_speed)))??;
        wait_ms(duration_ms);
        Ok(())
    })();

    let motor_stop_result = try_lock!(shared.m0 => stop_motor());
    let logger_stop_result = try_lock!(shared.logger => stop());

    move_status?;
    motor_stop_result??;

    /* ---------- Plot Firmware Log ---------- */
    let (log_dir, file_dir) = logger_stop_result??;

    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_speed_control(&csv_log, max_speed_pps, &pid_config)?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}
