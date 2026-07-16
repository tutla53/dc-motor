use super::*;

pub fn move_trapezoid(
    target_rotation: f64,
    speed_rpm: f64,
    acc_cps2: i32,
) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let current_pos = with_lock!(shared, m0.get_motor_pos())?;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    let log_mask = LogMask::CommandedPosition | LogMask::MotorPosition;
    let time_sampling = 1;

    with_lock!(shared, m0.clear_motor_event())?;

    with_lock!(shared, logger.start(log_mask, time_sampling))?;
    wait_ms(300);

    with_lock!(
        shared,
        m0.move_motor_pos_trapezoid(
            Position::from_rotation(target_rotation),
            Speed::from_rpm(speed_rpm),
            Acceleration::from_cps_sq(acc_cps2)
        )
    )?;

    with_lock!(shared, m0.wait_move_done(Duration::from_secs(20)))?;
    wait_ms(300);

    with_lock!(shared, logger.stop())?;
    wait_ms(300);

    with_lock!(shared, m0.stop_motor())?;

    let current_pos = with_lock!(shared, m0.get_motor_pos())?;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    Ok(())
}

pub fn move_speed(target_speed: f64) -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    let current_pos = with_lock!(shared, m0.get_motor_pos())?;
    println!(
        "Initial Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    let log_mask = LogMask::CommandedSpeed | LogMask::MotorSpeed;
    let time_sampling = 1;

    with_lock!(shared, m0.clear_motor_event())?;

    with_lock!(shared, logger.start(log_mask, time_sampling))?;
    wait_ms(300);

    with_lock!(shared, m0.move_motor_speed(Speed::from_rpm(target_speed)))?;

    wait_ms(1500);

    with_lock!(shared, logger.stop())?;
    wait_ms(300);

    with_lock!(shared, m0.stop_motor())?;

    let current_pos = with_lock!(shared, m0.get_motor_pos())?;
    println!(
        "Final Pos: {} count, {:.2} rotation",
        current_pos.count, current_pos.rotation
    );

    Ok(())
}
