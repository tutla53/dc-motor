// This is the place to create custom high-level motor experiment scripts.
// This module contains routines exposed through the `run` CLI command.
//
// Please read the `rust_script/README.md` for the detailed information.

use super::*;

pub fn enable_motor() -> DefaultResult<()> {
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

pub fn open_loop(percent: f64) -> DefaultResult<()> {
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

    let move_status = (|| -> DefaultResult<()> {
        try_lock!(shared.m0 => move_motor_open_loop(Pwm::from_percent(percent)))??;
        wait_ms(duration_ms);
        Ok(())
    })();

    let (log_dir, file_dir) = finalize_motor_routine(&shared.m0, &shared.logger, move_status)?;

    /* ---------- Plot Firmware Log ---------- */
    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_open_loop(&csv_log, ModelKind::Nonlinear)?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}

pub fn pos_trapezoid_move(
    target_rotation: f64,
    speed_rpm: f64,
    acc_pps2: i32,
) -> DefaultResult<()> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    /* ---------- Config ---------- */
    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;
    let chart_title = "Trapezoid Position Control";
    let y_label = "Position (rotation)";

    /* ---------- Gathering Motor Info ---------- */
    let initial_pos = try_lock!(shared.m0 => get_motor_pos())??;
    let pid_speed_config = try_lock!(shared.m0 => get_pid_motor_speed())??;
    let pid_pos_config = try_lock!(shared.m0 => get_pid_motor_pos())??;
    let max_speed_pps = try_lock!(shared.m0 => get_motor_max_speed())??;

    /* ---------- Estimate Move Time ---------- */
    let timeout_ms = get_move_timeout_ms(
        &initial_pos,
        &Position::from_rotation(target_rotation),
        &Speed::from_rpm(speed_rpm),
        &Acceleration::from_pps_sq(acc_pps2),
        max_speed_pps as i32,
    )?;

    /* ---------- Display Motor Info ---------- */
    println!("  [INFO] - Initial Pos: {initial_pos}");

    /* ---------- Move Motor ---------- */
    try_lock!(shared.m0 => clear_motor_event())?;
    try_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    let move_status = (|| -> DefaultResult<()> {
        try_lock!(
            shared.m0 =>
            move_motor_pos_trapezoid(
                Position::from_rotation(target_rotation),
                Speed::from_rpm(speed_rpm),
                Acceleration::from_pps_sq(acc_pps2)
            )
        )??;

        try_lock!(shared.m0 => wait_move_done(Duration::from_millis(timeout_ms)))??;
        wait_ms(300);

        Ok(())
    })();

    let (log_dir, file_dir) = finalize_motor_routine(&shared.m0, &shared.logger, move_status)?;

    /* ---------- Get Motor Pos ---------- */
    let current_pos = try_lock!(shared.m0 => get_motor_pos())??;
    println!("  [INFO] - Final Pos: {current_pos}");

    /* ---------- Plot Firmware Log ---------- */
    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_position_control(
        &csv_log,
        ModelKind::Nonlinear,
        initial_pos.pulse,
        max_speed_pps,
        &pid_speed_config,
        &pid_pos_config,
    )?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}

pub fn pos_step_move(target_rotation: f64) -> DefaultResult<()> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    /* ---------- Config ---------- */
    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;
    let chart_title = "Step Position Response";
    let y_label = "Position (rotation)";

    /* ---------- Gathering Motor Info ---------- */
    let initial_pos = try_lock!(shared.m0 => get_motor_pos())??;
    let pid_speed_config = try_lock!(shared.m0 => get_pid_motor_speed())??;
    let pid_pos_config = try_lock!(shared.m0 => get_pid_motor_pos())??;
    let max_speed_pps = try_lock!(shared.m0 => get_motor_max_speed())??;

    /* ---------- Estimate Move Time ---------- */
    let timeout_ms = get_move_timeout_ms(
        &initial_pos,
        &Position::from_rotation(target_rotation),
        &Speed::from_pps(max_speed_pps as i32),
        &Acceleration::from_pps_sq(1_000_000), // TODO: Move this value to the motor config
        max_speed_pps as i32,
    )?;

    /* ---------- Display Motor Info ---------- */
    println!("  [INFO] - Initial Pos: {initial_pos}");

    /* ---------- Move Motor ---------- */
    try_lock!(shared.m0 => clear_motor_event())?;
    try_lock!(shared.logger=> start(log_mask, time_sampling))??;
    wait_ms(300);

    let move_status = (|| -> DefaultResult<()> {
        try_lock!(
            shared.m0 => move_motor_pos_step(Position::from_rotation(target_rotation))
        )??;

        try_lock!(shared.m0 => wait_move_done(Duration::from_millis(timeout_ms)))??;
        wait_ms(300);

        Ok(())
    })();

    let (log_dir, file_dir) = finalize_motor_routine(&shared.m0, &shared.logger, move_status)?;

    /* ---------- Get Motor Pos ---------- */
    let current_pos = try_lock!(shared.m0 => get_motor_pos())??;
    println!("  [INFO] - Final Pos: {current_pos}");

    /* ---------- Plot Firmware Log ---------- */
    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_position_control(
        &csv_log,
        ModelKind::Nonlinear,
        initial_pos.pulse,
        max_speed_pps,
        &pid_speed_config,
        &pid_pos_config,
    )?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}

pub fn speed_move(target_speed: f64) -> DefaultResult<()> {
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

    let move_status = (|| -> DefaultResult<()> {
        try_lock!(shared.m0 => move_motor_speed(Speed::from_rpm(target_speed)))??;
        wait_ms(duration_ms);
        Ok(())
    })();

    let (log_dir, file_dir) = finalize_motor_routine(&shared.m0, &shared.logger, move_status)?;

    /* ---------- Plot Firmware Log ---------- */
    let csv_log = CsvProcessing::extract_information(
        &file_dir,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    let simulation = MotorSimulation::simulate_speed_control(
        &csv_log,
        ModelKind::Nonlinear,
        max_speed_pps,
        &pid_config,
    )?;

    plot::plot_log(&log_dir, &csv_log, chart_title, y_label, &[simulation])?;

    Ok(())
}
