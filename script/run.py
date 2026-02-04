import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time
import math

import Board.Pico as Board
import FWLogger.FWLogger as Logger
import BasicFunction.Motor as Motor
import Config.Motor0
import Event.Event as Event

yaml_path = "YAML/Pico_0_1_0.yaml"
p = Board.Pico(yaml_path)
m0 = Motor.MoveMotor(device = p, configfile = Config.Motor0)
logger = Logger.FWLogger(p)

def speed_test(speed_rpm, time_sampling = 10):
    logger.run(time_sampling=time_sampling, mask=10)
    m0.move_motor_speed(speed_rpm)
    wait_move_done()
    time.sleep(1)
    logger.stop()
    p.stop_motor(0)

def wait_move_done(timeout=15):
    start = time.perf_counter()
    elapsed = 0
    p.event_queue.clear()
    
    while elapsed < timeout:
        if len(p.event_queue) > 0:
            p.event_queue.pop()
            print(f"Move Done, Elapsed: {elapsed:.2f} s")
            return True
        time.sleep(0.1)
        elapsed = time.perf_counter() - start

    print("Timeout waiting for move done.")
    return False

def pos_trapezoid_test(position_rotation, speed, acc, time_sampling = 10):
    current_pos = m0.get_motor_pos()['pos_rotation']
    duration = calculate_motion_time(position_rotation-current_pos, speed, acc)
    print("Current Pos: ", current_pos)
    print(f"Duration (est): {duration:.2f}s")
    
    logger.run(time_sampling=time_sampling)
    
    m0.move_motor_pos_trapezoid(position_rotation, speed, acc)
    # wait_move_done(duration+10)
    time.sleep(5)
    p.stop_motor(0)
    logger.stop()

def pos_step_test(position_rotation, duration=3, time_sampling = 10):
    
    current_pos = m0.get_motor_pos()['pos_rotation']
    print("Current Pos: ", current_pos)
    
    logger.run(time_sampling=time_sampling)
    
    m0.move_motor_pos_step(position_rotation)
    # wait_move_done(duration+10)
    # time.sleep(5)
    p.stop_motor(0)
    logger.stop()

def open_loop_test(pwm, duration=3, time_sampling = 10):
    logger.run(time_sampling=time_sampling)
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


# event.run()

# while True:
#     p.get_motor_pos(0)
#     event.print_logged_data()
#     time.sleep(1)
# pos_trapezoid_test(10, 1000, 10000)