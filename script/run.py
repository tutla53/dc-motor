# Pico Script

import time
import os

import Board.Pico as Board
import BasicFunction.Motor as Motor
import Config.Motor0 as Motor0Config
import Config.Motor1 as Motor1Config
import Tool.plotter
import Tool.FileProcessing
from Tool.MotorSim import SystemModel
from Tool.visualize import *
from base_url import *

# -------------------------------------- Initialization -------------------------------------- #
yaml_path = base_url+"/DeviceOpFuncs/DCMotor.toml"
p = Board.Pico(yaml_path)
m0 = Motor.MoveMotor(device = p, configfile = Motor0Config)
m1 = Motor.MoveMotor(device = p, configfile = Motor1Config)

# -------------------------------------- Playground -------------------------------------- #
# TODO: Create the Information Guide

def verify_open_loop(mode: SystemModel = SystemModel.Linear):
    
    pid_pos_config      = [0, 0, 0, 0]
    
    assets_dir = base_url[:-7] + "/assets/01_System_Identification/open-loop-responses/"
    saved_dir = base_url[:-7] + "/assets/01_System_Identification/open-loop-plot/"
    
    all_log_files = os.listdir(assets_dir)
        
    dt_s = 0.005 # Default Time Sampling
    
    for idx, filename in enumerate(all_log_files):
        if ".csv" not in filename: continue
        
        motor_response = Tool.FileProcessing.extract_firmware_log_data(assets_dir+filename, "Commanded_PWM", "Motor_Speed(RPM)")
        
        if motor_response["start_time_s"] is None:
            motor_response["start_time_s"] = 0
            motor_response["target"] = 0
        
        simulation_time_list, simulation_speed, simulation_target = m0.sim.simulate_open_loop_response(
                                                                target_pwm  = motor_response["target"],
                                                                u           = motor_response["command_list"],
                                                                mode        = mode,
                                                                )
        
        percent_pwm = round((motor_response["target"]/m0.config.MAX_PWM_TICKS)*100)
        file_tag = f"A_{percent_pwm}_{idx}_" if motor_response["target"]>0 else f"B_{percent_pwm}_{idx}_"
        
        Tool.plotter.create_simulation_plot(log_dir                     = saved_dir, 
                                            pid_config                  = pid_pos_config,
                                            simulation_time_list        = simulation_time_list, 
                                            actual_time_list            = motor_response["time_list"],
                                            simulation_data_list        = simulation_speed, 
                                            simulation_commanded_list   = simulation_target,  
                                            actual_data_list            = motor_response["data_list"], 
                                            actual_commanded_list       = None,
                                            tag                         = file_tag,
                                            plot_title                  = f"PWM Input: {percent_pwm}%",
                                            y_label                     = "Motor Speed (RPM)",
                                            actual_data_label           = "Actual Speed",
                                            actual_commanded_label      = "Commanded PWM",
                                            simulation_data_label       = "Simulation Speed",
                                            simulation_commanded_label  = "Commanded PWM",
                                            same_unit                   = False,
                                            )