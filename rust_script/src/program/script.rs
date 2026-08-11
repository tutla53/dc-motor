use super::*;

// TODO: Add the guidline to create the custom script

pub fn open_loop(pwm: i32) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    // Open Loop Config
    let log_mask = LogMask::CommandedPwm | LogMask::MotorSpeed;
    let time_sampling = 1;
    let chart_title = "Open Loop Response";
    let y_label = "PWM (ticks), Velocity (RPM)";
    let duration_ms = 1500;

    // Clear Motor Event
    run_with_lock!(shared.m0 => clear_motor_event())?;

    // Start Firmware Logger
    run_with_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    // Move Motor
    run_with_lock!(shared.m0 => move_motor_open_loop(pwm))?;
    wait_ms(duration_ms);

    // Stop Firmware Logger
    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    // Stop Motor
    run_with_lock!(shared.m0 => stop_motor())?;

    // Plot Firmware Log
    plot::plot_csv(
        &log_dir,
        &file_dir,
        &chart_title,
        &y_label,
    )?;

    Ok(())
}

pub fn pos_trapezoid_move(
    target_rotation: f64,
    speed_rpm: f64,
    acc_cps2: i32,
) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");
    
    // Position Control Config
    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;
    let chart_title = "Trapezoid Position Control";
    let y_label = "Position (rotation)";

    // Get Motor Pos
    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    // Clear Motor Event
    run_with_lock!(shared.m0 => clear_motor_event())?;

    // Start Firmware Logger
    run_with_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    // Move Motor
    run_with_lock!(
        shared.m0 =>
        move_motor_pos_trapezoid(
            Position::from_rotation(target_rotation),
            Speed::from_rpm(speed_rpm),
            Acceleration::from_cps_sq(acc_cps2)
        )
    )?;

    // Wait Motor to Reach the Target Position
    if let Err(e) = run_with_lock!(shared.m0 => wait_move_done(Duration::from_secs(20)))? {
        println!("{}", e);
    }
    wait_ms(300);

    // Stop the Firmware Logger
    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    // Stop Motor
    run_with_lock!(shared.m0 => stop_motor())?;

    // Get Current Pos
    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    // Plot Firmware Log    
    plot::plot_csv(
        &log_dir,
        &file_dir,
        &chart_title,
        &y_label,
    )?;

    Ok(())
}

pub fn pos_step_move(target_rotation: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    // Position Control Config
    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;
    let chart_title = "Step Position Response";
    let y_label = "Position (rotation)";
    
    // Get Current Pos
    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    // Clear Motor Event
    run_with_lock!(shared.m0 => clear_motor_event())?;

    // Start Firmware Logger
    run_with_lock!(shared.logger=> start(log_mask, time_sampling))??;
    wait_ms(300);

    // Move Motor
    run_with_lock!(
        shared.m0 => move_motor_pos_step(Position::from_rotation(target_rotation))
    )?;

     // Wait Motor to Reach the Target Position
    if let Err(e) = run_with_lock!(shared.m0 => wait_move_done(Duration::from_secs(20)))? {
        println!("{}", e);
    }
    wait_ms(300);

    // Stop Firmware Logger
    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    // Stop Motor
    run_with_lock!(shared.m0 => stop_motor())?;

    // Get Current Pos
    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    // Plot Firmware Log
    plot::plot_csv(
        &log_dir,
        &file_dir,
        &chart_title,
        &y_label,
    )?;

    Ok(())
}

pub fn speed_move(target_speed: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    // Speed Control Config
    let log_mask = LogMask::CommandedSpeed | LogMask::MotorSpeed;
    let time_sampling = 1;
    let chart_title = "Closed Loop Velocity Response";
    let y_label = "Velocity (RPM)";
    let duration_ms = 1500;

    // Clear Motor Event
    run_with_lock!(shared.m0 => clear_motor_event())?;

    // Start Fimware Logger
    run_with_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    // Move Motor
    run_with_lock!(shared.m0 => move_motor_speed(Speed::from_rpm(target_speed)))?;
    wait_ms(duration_ms);

    // Stop Firmware Logger
    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    // Stop Motor
    run_with_lock!(shared.m0 => stop_motor())?;

    // Plot Firmware Logger
    plot::plot_csv(
        &log_dir,
        &file_dir,
        &chart_title,
        &y_label,
    )?;

    Ok(())
}
