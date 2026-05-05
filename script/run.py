# Pico Script

import time
import os

import Board.Pico as Board
import BasicFunction.Motor as Motor
import Config.Motor0
import Config.Motor1
import Tool.plotter
import Tool.FileProcessing
from Tool.visualize import *
from base_url import *

# -------------------------------------- Initialization -------------------------------------- #
yaml_path = base_url+"/YAML/Pico_0_1_3.yaml"
p = Board.Pico(yaml_path)
m0 = Motor.MoveMotor(device = p, configfile = Config.Motor0)
m1 = Motor.MoveMotor(device = p, configfile = Config.Motor1)

# -------------------------------------- Playground -------------------------------------- #
def speed_test(motor_id, speed_rpm, time_sampling = 10, timeout= 10):
    if motor_id == 0:
        m = m0
    elif motor_id == 1:
        m = m1
    else:
        printr("Invalid Motor ID")
        return
    
    tag = "Speed_Step_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    m.start_log(mask=10, time_sampling=time_sampling, folder_tag=tag)
    m.move_motor_speed(speed_rpm)
    time.sleep(1)
    m.stop_motor()
    file_tag = m.stop_log()
    
    Tool.plotter.save_image_by_tag(file_tag)

def pos_trapezoid_test(motor_id, position_rotation, speed, acc, time_sampling = 10):
    if motor_id == 0:
        m = m0
    elif motor_id == 1:
        m = m1
    else:
        printr("Invalid Motor ID")
        return
    
    tag = "Trapezoid_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    
    m.start_log(mask=5,time_sampling=time_sampling, folder_tag=tag)
    m.move_motor_pos_trapezoid(position_rotation, speed, acc, print_motor_log=True)
    m.stop_motor()
    time.sleep(0.5)
    file_tag = m.stop_log()
    
    Tool.plotter.save_image_by_tag(file_tag)

def pos_step_test(motor_id, position_rotation, duration=3, time_sampling = 10):
    if motor_id == 0:
        m = m0
    elif motor_id == 1:
        m = m1
    else:
        printr("Invalid Motor ID")
        return
    
    tag = "Pos_Step_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    
    m.start_log(mask = 5, time_sampling=time_sampling, folder_tag=tag)
    m.move_motor_pos_step(position_rotation, print_motor_log=True)
    time.sleep(duration)
    m.stop_motor()
    file_tag = m.stop_log()
    
    Tool.plotter.save_image_by_tag(file_tag)

def open_loop_test(motor_id, pwm, duration=3, time_sampling = 10):
    if motor_id == 0:
        m = m0
    elif motor_id == 1:
        m = m1
    else:
        printr("Invalid Motor ID")
        return
    
    tag = "Open_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    
    m.start_log(mask = 18, time_sampling=time_sampling, folder_tag=tag)
    m.move_motor_open_loop(pwm)
    time.sleep(duration)
    m.stop_motor()
    file_tag = m.stop_log()
    
    Tool.plotter.save_image_by_tag(file_tag)

def speed_test_with_simulation(motor_id, speed_rpm, time_sampling = 5):
    motor = None
    
    if motor_id == 0:
        motor = m0
    elif motor_id == 1:
        motor = m1
    else:
        printr("Invalid Motor ID")
        return
    
    pid_config = p.get_pid_motor_speed(motor_id)
    
    tag = "Speed_Step_and_Simulation" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    motor.start_log(mask=10, time_sampling=time_sampling, folder_tag=tag)
    motor.move_motor_speed(speed_rpm)
    time.sleep(1)
    motor.stop_motor()
    dir_tag = motor.stop_log()
    
    log_dir = base_url+"/LOG/"+dir_tag+"/"
    filename = os.listdir(base_url+"/LOG/"+dir_tag)[0]

    extracted_data = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, "Commanded_Speed(RPM)", "Motor_Speed(RPM)")
    target, start_time, duration, dt_s, actual_commanded, actual_speed, actual_time = extracted_data
    
    simulation_time_list, simulation_speed, simulation_target = motor.simulate_pid_speed_control(
                                                    target=target, 
                                                    start_time=start_time, 
                                                    duration=duration)
    
    Tool.plotter.create_simulation_plot(log_dir, 
                                        pid_config,
                                        simulation_time_list, 
                                        actual_time,
                                        simulation_speed, 
                                        simulation_target,  
                                        actual_speed, 
                                        actual_commanded)

def simulate_speed_control(speed_rpm, kp, ki, kd, i_limit, start_time = 0.4, duration=1.5):
    
    pid_config = {
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "i_limit": i_limit,
    }
    
    tag = "Simulation_Speed_Step_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    log_dir = base_url+"/LOG/"+ tag+"/"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    simulation_time_list, simulation_speed, simulation_target = m0.simulate_pid_speed_control(
                                                    target=speed_rpm, 
                                                    start_time=start_time, 
                                                    duration=duration,
                                                    pid_config=pid_config)
    Tool.plotter.create_simulation_plot(log_dir, 
                                        pid_config,
                                        simulation_time_list, 
                                        None,
                                        simulation_speed, 
                                        simulation_target,  
                                        None, 
                                        None)
