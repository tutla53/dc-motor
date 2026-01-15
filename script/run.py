import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time
import math

import Board.Pico as Board
import FWLogger.FWLogger as Logger
import BasicFunction.Motor as Motor
import Config.Motor0

yaml_path = "YAML/Pico_0_0_6.yaml"
p = Board.Pico(yaml_path)
m0 = Motor.MoveMotor(device = p, configfile = Config.Motor0)
logger = Logger.FWLogger(p)

def speed_test(speed_rpm, time_sampling = 5):
    logger.run(time_sampling=time_sampling, mask=10)
    m0.move_motor_speed(speed_rpm)
    time.sleep(3)
    logger.stop()
    p.stop_motor(0)

def pos_trapezoid_test(position_rotation, speed, acc, time_sampling=5):
    current_pos = m0.get_motor_pos()['pos_rotation']
    duration = calculate_motion_time(position_rotation-current_pos, speed, acc) + 1
    print(f"Duration (est): {duration:.2f}s")

    logger.run(time_sampling=time_sampling, mask=5)
    m0.move_motor_pos_trapezoid(position_rotation, speed, acc)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)

def pos_step_test(position_rotation, duration=3, time_sampling=5):
    logger.run(time_sampling=time_sampling, mask=5)
    m0.move_motor_pos_step(position_rotation)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)

def open_loop_test(pwm, duration=3, time_sampling=5):
    logger.run(time_sampling=time_sampling, mask=18)
    m0.move_motor_open_loop(pwm)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)

def encoder_check():
    while True:
        print(m0.get_motor_pos())
        time.sleep(0.1)

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

def save_image(path="LOG/20250409103638.csv"):
    data = pd.read_csv(path)
    t = data["Timestamp"]/1000.0
    column_names = []
    tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    plot_mode = "Position"

    for column in data.columns:
        if column == "Timestamp":
            continue
        if "Speed" in column:
            plot_mode = "Speed"
        column_names.append(column)

    for column in column_names:
        plt.plot(t, data[column], label = column)
    
    if plot_mode == "Position":
        plt.ylabel("Position (Rotation)")
        plt.title("Position Response")

    elif plot_mode == "Speed":
        plt.ylabel("Speed (RPM)")
        plt.title("Speed Response")

    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid()
    plt.savefig("LOG/IMG/result_"+tag+".jpg", dpi = 300)
    plt.close()