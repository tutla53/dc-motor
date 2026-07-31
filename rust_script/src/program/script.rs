use super::*;

// TODO: Add the guidline to create the custom script

pub fn open_loop(pwm: i32) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let log_mask = LogMask::CommandedPwm | LogMask::MotorSpeed;
    let time_sampling = 1;

    run_with_lock!(shared.m0 => clear_motor_event())?;

    run_with_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    run_with_lock!(shared.m0 => move_motor_open_loop(pwm))?;
    wait_ms(1500);

    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;

    wait_ms(300);

    run_with_lock!(shared.m0 => stop_motor())?;

    plotter::plot_csv(&log_dir, &file_dir, "Open Loop Response", "PWM (ticks), Velocity (RPM)")?;

    Ok(())
}

pub fn pos_trapezoid_move(
    target_rotation: f64,
    speed_rpm: f64,
    acc_cps2: i32,
) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;

    run_with_lock!(shared.m0 => clear_motor_event())?;

    run_with_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    run_with_lock!(
        shared.m0 =>
        move_motor_pos_trapezoid(
            Position::from_rotation(target_rotation),
            Speed::from_rpm(speed_rpm),
            Acceleration::from_cps_sq(acc_cps2)
        )
    )?;

    if let Err(e) = run_with_lock!(shared.m0 => wait_move_done(Duration::from_secs(20)))? {
        println!("{}", e);
    }

    wait_ms(300);

    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    run_with_lock!(shared.m0 => stop_motor())?;

    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    plotter::plot_csv(&log_dir, &file_dir, "Trapezoid Position Control", "Position (rotation)")?;

    Ok(())
}

pub fn pos_step_move(target_rotation: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;

    run_with_lock!(shared.m0 => clear_motor_event())?;

    run_with_lock!(shared.logger=> start(log_mask, time_sampling))??;
    wait_ms(300);

    run_with_lock!(
        shared.m0 => move_motor_pos_step(Position::from_rotation(target_rotation))
    )?;

    if let Err(e) = run_with_lock!(shared.m0 => wait_move_done(Duration::from_secs(20)))? {
        println!("{}", e);
    }
    wait_ms(300);

    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    run_with_lock!(shared.m0 => stop_motor())?;

    let current_pos = run_with_lock!(shared.m0 => get_motor_pos())??;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    plotter::plot_csv(&log_dir, &file_dir, "Step Position Response", "Position (rotation)")?;

    Ok(())
}

pub fn speed_move(target_speed: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let log_mask = LogMask::CommandedSpeed | LogMask::MotorSpeed;
    let time_sampling = 1;

    run_with_lock!(shared.m0 => clear_motor_event())?;

    run_with_lock!(shared.logger => start(log_mask, time_sampling))??;
    wait_ms(300);

    run_with_lock!(shared.m0 => move_motor_speed(Speed::from_rpm(target_speed)))?;

    wait_ms(1500);

    let (log_dir, file_dir) = run_with_lock!(shared.logger => stop())??;
    wait_ms(300);

    run_with_lock!(shared.m0 => stop_motor())?;

    plotter::plot_csv(&log_dir, &file_dir, "Closed Loop Velocity Response", "Velocity (RPM)")?;

    Ok(())
}
