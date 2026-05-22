# Pico Script

import time
import os

import Board.Pico as Board
import BasicFunction.Motor as Motor
import Config.Motor0 as Motor0Config
import Config.Motor1 as Motor1Config
import Tool.plotter
import Tool.FileProcessing
from Tool.visualize import *
from base_url import *

# -------------------------------------- Initialization -------------------------------------- #
yaml_path = base_url+"/DeviceOpFuncs/DCMotor.toml"
p = Board.Pico(yaml_path)
m0 = Motor.MoveMotor(device = p, configfile = Motor0Config)
m1 = Motor.MoveMotor(device = p, configfile = Motor1Config)

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

def pos_test_with_simulation(motor_id, position_rotation, duration=1.5, time_sampling = 5):
    motor = None
    
    if motor_id == 0:
        motor = m0
    elif motor_id == 1:
        motor = m1
    else:
        printr("Invalid Motor ID")
        return
    
    pid_speed = p.get_pid_motor_speed(motor_id)
    pid_speed_config = [pid_speed["kp"], pid_speed["ki"], pid_speed["kd"], pid_speed["i_limit"]]
    
    pid_pos = p.get_pid_motor_pos(motor_id)
    pid_pos_config = [pid_pos["kp"], pid_pos["ki"], pid_pos["kd"], pid_pos["i_limit"]] 
    
    tag = "Speed_Step_and_Simulation" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    motor.start_log(mask = 5, time_sampling=time_sampling, folder_tag=tag)
    motor.move_motor_pos_step(position_rotation, print_motor_log=True)
    time.sleep(duration)
    motor.stop_motor()
    dir_tag = motor.stop_log()
    
    log_dir = base_url+"/LOG/"+dir_tag+"/"
    filename = os.listdir(base_url+"/LOG/"+dir_tag)[0]
    
    motor_response = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, "Commanded_Position(rotation)", "Motor_Position(rotation)", threshold_percent=2.0)
    
    normalized_actual_position_list = []
    
    for pos in motor_response["data_list"]:
        normalized_actual_position_list.append(pos-motor_response["command_list"][0])
    
    simulation_time_list, simulation_pos, simulation_target, simulation_speed = m0.sim.simulate_pid_pos_control(
                                                            pid_pos_params      = pid_pos_config,
                                                            pid_speed_params    = pid_speed_config,
                                                            target_rotation     = position_rotation - motor_response["command_list"][0], 
                                                            start_time          = motor_response["start_time_s"],
                                                            duration            = motor_response["duration_s"],
                                                            )
    
    Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                        pid_config                  = pid_pos_config,
                                        simulation_time_list        = simulation_time_list, 
                                        actual_time_list            = motor_response["time_list"],
                                        simulation_data_list        = simulation_pos, 
                                        simulation_commanded_list   = simulation_target,  
                                        actual_data_list            = normalized_actual_position_list, 
                                        actual_commanded_list       = None,
                                        tag                         = "Position_",
                                        plot_title                  = "",
                                        y_label                     = "Motor Position (Rotation)",
                                        actual_data_label           = "Actual Position",
                                        actual_commanded_label      = "",
                                        simulation_data_label       = "Simulation Position",
                                        simulation_commanded_label  = "Commanded Position"
                                        )

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
    
    kp = pid_config["kp"]
    ki = pid_config["ki"]
    kd = pid_config["kd"]
    i_limit = pid_config["i_limit"]
    
    pid_params = [kp, ki, kd, i_limit]
    
    tag = "Speed_Step_and_Simulation" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    motor.start_log(mask=10, time_sampling=time_sampling, folder_tag=tag)
    motor.move_motor_speed(speed_rpm)
    time.sleep(1)
    motor.stop_motor()
    dir_tag = motor.stop_log()
    
    log_dir = base_url+"/LOG/"+dir_tag+"/"
    filename = os.listdir(base_url+"/LOG/"+dir_tag)[0]

    motor_response = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, "Commanded_Speed(RPM)", "Motor_Speed(RPM)")
    
    simulation_time_list, simulation_speed, simulation_target = motor.sim.simulate_pid_speed_control(
                                                    pid_params=pid_params,
                                                    target_rpm=motor_response["target"], 
                                                    start_time=motor_response["start_time_s"], 
                                                    duration=motor_response["duration_s"])
    
    Tool.plotter.create_simulation_plot(log_dir                 = log_dir, 
                                    pid_config                  = pid_params,
                                    simulation_time_list        = simulation_time_list, 
                                    actual_time_list            = motor_response["time_list"],
                                    simulation_data_list        = simulation_speed, 
                                    simulation_commanded_list   = simulation_target,  
                                    actual_data_list            = motor_response["data_list"], 
                                    actual_commanded_list       = None,
                                    tag                         = "Speed_",
                                    plot_title                  = "",
                                    y_label                     = "Motor Speed (RPM)",
                                    actual_data_label           = "Actual Speed",
                                    actual_commanded_label      = "",
                                    simulation_data_label       = "Simulation Speed",
                                    simulation_commanded_label  = "Commanded Speed"
                                    )

def simulate_speed_control(speed_rpm, kp, ki, kd, i_limit, start_time = 0.4, duration=1.5):
    
    pid_config = [kp, ki, kd, i_limit]
    
    tag = "Simulation_Speed_Step_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    log_dir = base_url+"/LOG/"+ tag+"/"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    simulation_time_list, simulation_speed, simulation_target = m0.sim.simulate_pid_speed_control(
                                                    pid_params=pid_config,
                                                    target_rpm=speed_rpm, 
                                                    start_time=start_time, 
                                                    duration=duration,
                                                    )
    
    Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                        pid_config                  = pid_config,
                                        simulation_time_list        = simulation_time_list, 
                                        actual_time_list            = None,
                                        simulation_data_list        = simulation_speed, 
                                        simulation_commanded_list   = simulation_target,  
                                        actual_data_list            = None, 
                                        actual_commanded_list       = None,
                                        tag                         = "",
                                        plot_title                  = "",
                                        y_label                     = "Motor Speed (RPM)",
                                        actual_data_label           = "",
                                        actual_commanded_label      = "",
                                        simulation_data_label       = "Simulation Speed",
                                        simulation_commanded_label  = "Commanded Speed"
                                        )

def simulate_pos_control(pos_rotation, start_time = 0.4, duration=1.5):
    
    pid_pos_config      = [25, 0, 5, 1500]
    pid_speed_config    = [7.04482, 0.94301, 8.3334, 5319]
    
    tag = "Simulation_Position_Step_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    log_dir = base_url+"/LOG/"+ tag+"/"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    simulation_time_list, simulation_pos, simulation_target, simulation_speed = m0.sim.simulate_pid_pos_control(
                                                    pid_pos_params=pid_pos_config,
                                                    pid_speed_params=pid_speed_config,
                                                    target_rotation=pos_rotation, 
                                                    start_time=start_time, 
                                                    duration=duration,
                                                    )
    
    Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                        pid_config                  = pid_pos_config,
                                        simulation_time_list        = simulation_time_list, 
                                        actual_time_list            = None,
                                        simulation_data_list        = simulation_pos, 
                                        simulation_commanded_list   = simulation_target,  
                                        actual_data_list            = None, 
                                        actual_commanded_list       = None,
                                        tag                         = "",
                                        plot_title                  = "",
                                        y_label                     = "Motor Position (Rotation)",
                                        actual_data_label           = "",
                                        actual_commanded_label      = "",
                                        simulation_data_label       = "Simulation Position",
                                        simulation_commanded_label  = "Commanded Position"
                                        )

def simulate_open_loop(target, start_time = 0.4, duration=1.5):
    
    pid_pos_config      = [0, 0, 0, 0]
    
    tag = "Simulation_Open_Loop_Test_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    log_dir = base_url+"/LOG/"+ tag+"/"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    simulation_time_list, simulation_speed_list, commanded_pwm_list = m0.sim.simulate_open_loop_response(
                                                                    target_pwm=target, 
                                                                    start_time=start_time, 
                                                                    duration=duration
                                                                    )
    
    Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                        pid_config                  = pid_pos_config,
                                        simulation_time_list        = simulation_time_list, 
                                        actual_time_list            = None,
                                        simulation_data_list        = simulation_speed_list, 
                                        simulation_commanded_list   = commanded_pwm_list,  
                                        actual_data_list            = None, 
                                        actual_commanded_list       = None,
                                        tag                         = "",
                                        plot_title                  = f"Open Loop Simulation with PWM Target {target}",
                                        y_label                     = "Motor Speed (RPM)",
                                        actual_data_label           = "",
                                        actual_commanded_label      = "",
                                        simulation_data_label       = "Simulation Speed",
                                        simulation_commanded_label  = "Input PWM"
                                        )

def verify_open_loop():
    
    pid_pos_config      = [0, 0, 0, 0]
    
    assets_dir = base_url + "/assets/open-loop-responses/"
    saved_dir = base_url + "/assets/open-loop-plot/"
    
    all_log_files = os.listdir(assets_dir)
        
    dt_s = 0.005 # Default Time Sampling
    
    for idx, filename in enumerate(all_log_files):
        if ".csv" not in filename: continue
        
        motor_response = Tool.FileProcessing.extract_firmware_log_data(assets_dir+filename, "Commanded_PWM", "Motor_Speed(RPM)")

        simulation_time_list, simulation_speed_list, commanded_pwm_list = m0.sim.simulate_open_loop_response(
                                                                        target_pwm  =   motor_response["target"], 
                                                                        start_time  =   motor_response["start_time_s"], 
                                                                        duration    =   motor_response["duration_s"]
                                                                        )
        
        percent_pwm = round((motor_response["target"]/m0.config.MAX_PWM_TICKS)*100)
        file_tag = f"A_{percent_pwm}_{idx}_" if motor_response["target"]>0 else f"B_{percent_pwm}_{idx}_"
        
        Tool.plotter.create_simulation_plot(log_dir                     = saved_dir, 
                                            pid_config                  = pid_pos_config,
                                            simulation_time_list        = simulation_time_list, 
                                            actual_time_list            = motor_response["time_list"],
                                            simulation_data_list        = simulation_speed_list, 
                                            simulation_commanded_list   = None,  
                                            actual_data_list            = motor_response["data_list"], 
                                            actual_commanded_list       = motor_response["command_list"],
                                            tag                         = file_tag,
                                            plot_title                  = f"PWM Input: {percent_pwm}%",
                                            y_label                     = "Motor Speed (RPM)",
                                            actual_data_label           = "Actual Speed",
                                            actual_commanded_label      = "Commanded PWM",
                                            simulation_data_label       = "Simulation Speed",
                                            simulation_commanded_label  = ""
                                            )