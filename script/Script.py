from Device.Device import *
import time
import math

def speed_test(speed_rpm, time_sampling = 5):
    logger.run(time_sampling=time_sampling, mask=10)
    motor0.move_motor_speed(speed_rpm)
    time.sleep(3)
    logger.stop()
    p.stop_motor(0)

def pos_trapezoid_test(position_rotation, speed, acc, time_sampling=5):
    current_pos = motor0.get_motor_pos()['pos_rotation']
    duration = calculate_motion_time(position_rotation-current_pos, speed, acc) + 2

    logger.run(time_sampling=time_sampling, mask=5)
    motor0.move_motor_pos_trapezoid(position_rotation, speed, acc)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)

def pos_step_test(position_rotation, duration=3, time_sampling=5):
    logger.run(time_sampling=time_sampling, mask=5)
    motor0.move_motor_pos_step(position_rotation)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)

def calculate_motion_time(target_distance, speed_rpm, acceleration_rpm_per_s):
    # Convert RPM and RPM/s to rotations per second and rotations per second squared
    v_max = speed_rpm / 60.0  # rotations per second
    a_max = acceleration_rpm_per_s / 60.0  # rotations per second squared
    
    displacement_abs = abs(target_distance)
    
    # Handle edge cases with zero acceleration or zero velocity
    if a_max <= 0:
        if v_max <= 0:
            return 0.0  # No movement possible
        else:
            return displacement_abs / v_max
    
    # Calculate minimum distance required for trapezoidal profile
    d_min = (v_max ** 2) / a_max
    
    if displacement_abs >= d_min:
        # Trapezoidal profile
        t_acc = v_max / a_max
        t_coast = (displacement_abs - d_min) / v_max
        t_total = 2 * t_acc + t_coast
    else:
        # Triangular profile
        t_acc = math.sqrt(displacement_abs / a_max)
        t_total = 2 * t_acc
    
    return t_total