import time
import os
import Tool.FileProcessing
import Tool.plotter
import Tool.SystemIdentification
import Tool.MotorSim
import numpy as np

from Config.Data import *
from BasicFunction.Motor import MoveMotor
from base_url import *
from Tool.visualize import *
from Tool.MotorSim import SystemModel

class MotorControl:
    def __init__(self, move_motor: MoveMotor):
        self.core = move_motor
        self.simulate = Tool.MotorSim.SimulateMotor(move_motor.config)
        self.optimize = Tool.SystemIdentification.MotorOptimization(move_motor.config)

    def move_open_loop(self, pwm, duration=1.0, time_sampling = 1, simulation_mode: SystemModel = SystemModel.Nonlinear):
        move_name = "open"
        tag = group_name[move_name] + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time())) 
                
        self.core.start_log(mask = 18, time_sampling=time_sampling, folder_tag=tag)
        self.core.move_motor_open_loop(pwm)
        time.sleep(duration)
        self.core.stop_motor()
        dir_name = self.core.stop_log()
        
        self.plot_firmware_log(dir_name, move_name, simulation_mode)
        
    def move_speed(self, speed_rpm, time_sampling = 1, simulation_mode: SystemModel = SystemModel.Nonlinear):
        move_name = "speed"
        tag = group_name[move_name] + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))        
        
        self.core.start_log(mask=10, time_sampling=time_sampling, folder_tag=tag)
        self.core.move_motor_speed(speed_rpm)
        time.sleep(1)
        self.core.stop_motor()
        dir_name = self.core.stop_log()
        
        self.plot_firmware_log(dir_name, move_name, simulation_mode)

    def move_pos(self, position_rotation, speed_rpm=None, acc_rpm=None, duration=1.5, time_sampling = 1, simulation_mode: SystemModel = SystemModel.Nonlinear):
        move_name = "pos"
        tag = group_name[move_name] + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        
        self.core.start_log(mask = 5, time_sampling=time_sampling, folder_tag=tag)
        if speed_rpm == None or acc_rpm == None:
            self.core.move_motor_pos_step(position_rotation, print_motor_log=True)
        else:
            self.core.move_motor_pos_trapezoid(position_rotation, speed_rpm=speed_rpm, acc_rpm=acc_rpm, print_motor_log=True)
        time.sleep(duration)
        self.core.stop_motor()
        dir_name = self.core.stop_log()
                
        self.plot_firmware_log(dir_name, move_name, simulation_mode)
        
    def plot_firmware_log(self, dir_name, move_name, simulation_mode):
        pid_speed = self.core.dev.get_pid_motor_speed(self.core.motor_id)
        pid_pos = self.core.dev.get_pid_motor_pos(self.core.motor_id)
                
        pid_speed_config = [pid_speed["kp"], pid_speed["ki"], pid_speed["kd"], pid_speed["i_limit"]]
        pid_pos_config = [pid_pos["kp"], pid_pos["ki"], pid_pos["kd"], pid_pos["i_limit"]] 
        
        log_dir = base_url+"/LOG/"+dir_name+"/"
        filename = os.listdir(base_url+"/LOG/"+dir_name)[0]
        
        motor_response = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, command_header[move_name], data_header[move_name], threshold_percent=2.0)
        
        initial_position = motor_response["command_list"][0]
        
        # Simulation
        if move_name == "pos":
            result = self.simulate.simulate_pid_pos_control(
                        pid_pos_params      = pid_pos_config,
                        pid_speed_params    = pid_speed_config,
                        initial_pos         = initial_position/self.core.config.ROTATION_PER_PULSE,
                        setpoint            = motor_response["command_list"]/self.core.config.ROTATION_PER_PULSE,
                        mode                = simulation_mode,
                        )
        elif move_name == "speed":
            if motor_response["start_time_s"] is not None:
                result = self.simulate.simulate_pid_speed_control(
                            pid_params      = pid_speed_config,
                            initial_speed   = motor_response["command_list"][0]/self.core.config.ROTATION_PER_PULSE/60,
                            setpoint        = motor_response["command_list"]/self.core.config.ROTATION_PER_PULSE/60,
                            mode            = simulation_mode
                            )
        elif move_name == "open":
            if motor_response["start_time_s"] is not None:
                result = self.simulate.simulate_open_loop_response(
                            u       = motor_response["command_list"],
                            mode    = simulation_mode,
                            )            
        
        # Create the Plot
        Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                            pid_config                  = pid_pos_config,
                                            simulation_time_list        = result[0], 
                                            actual_time_list            = motor_response["time_list"],
                                            simulation_data_list        = result[1], 
                                            simulation_commanded_list   = result[2],  
                                            actual_data_list            = motor_response["data_list"], 
                                            actual_commanded_list       = None,
                                            tag                         = group_name[move_name],
                                            plot_title                  = plot_title[move_name],
                                            y_label                     = y_label[move_name],
                                            actual_data_label           = data_legend[move_name],
                                            actual_commanded_label      = commanded_legend[move_name],
                                            simulation_data_label       = simulation_legend[move_name],
                                            simulation_commanded_label  = commanded_legend[move_name],
                                            same_unit                   = same_unit[move_name], 
                                            )
        
    def verify_open_loop(self, mode: SystemModel = SystemModel.Linear):
        
        pid_pos_config      = [0, 0, 0, 0]
        
        assets_dir = base_url[:-7] + "/assets/01_System_Identification/open-loop-responses/"
        saved_dir = base_url[:-7] + "/assets/01_System_Identification/open-loop-plot/"
        
        all_log_files = os.listdir(assets_dir)
        
        for idx, filename in enumerate(all_log_files):
            if ".csv" not in filename: continue
            
            motor_response = Tool.FileProcessing.extract_firmware_log_data(assets_dir+filename, "Commanded_PWM", "Motor_Speed(RPM)")
            
            if motor_response["start_time_s"] is None:
                motor_response["start_time_s"] = 0
                motor_response["target"] = 0
            
            simulation_time_list, simulation_speed, simulation_target = self.simulate.simulate_open_loop_response(
                                                                    target_pwm  = motor_response["target"],
                                                                    u           = motor_response["command_list"],
                                                                    mode        = mode,
                                                                    )
            
            percent_pwm = round((motor_response["target"]/self.core.config.MAX_PWM_TICKS)*100)
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