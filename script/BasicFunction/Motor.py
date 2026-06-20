import time
import math
import numpy as np
import os
import FWLogger.FWLogger as Logger
import Tool.SystemIdentification
import Tool.MotorSim
import Tool.FileProcessing
import Tool.plotter
from base_url import *

from Tool.visualize import *
from Tool.MotorSim import SystemModel

class MoveMotor:
    def __init__(self, device, configfile):
        self.dev = device
        self.config = configfile
        self.motor_id = configfile.motor_id
        self.logger = Logger.FWLogger(device)
        self.sim = Tool.MotorSim.SimulateMotor(configfile)
        self.optimize = Tool.SystemIdentification.MotorOptimization(configfile)

    def __wait_move_done(self, timeout=180):
        start = time.perf_counter()
        elapsed = 0
        self.dev.event_queue.clear()
        
        while elapsed < timeout:
            if len(self.dev.event_queue) > 0:
                event = self.dev.event_queue.pop()
                event_str = "MOVE_MOTOR_DONE"
                if event_str in event:
                    if event[event_str] == self.motor_id:
                        print_log("MOVE DONE", f"Move Done, Elapsed: ", end = "")
                        printg(f"{elapsed:.2f} s")
                        return True
            time.sleep(0.1)
            elapsed = time.perf_counter() - start

        printr("[TIMEOUT] waiting for move done.")
        return False
    
    def start_log(self, mask, time_sampling, folder_tag = ""):
        self.logger.run(motor_id = self.motor_id, mask=mask, time_sampling=time_sampling, folder_tag=folder_tag)
    
    def stop_log(self):
        stat = self.logger.stop(self.motor_id)
        return stat
    
    def print_motor_info(self, id, current_pos, duration, timeout):
        print_log(f"MOVE MOTOR: {id}", "Initial Position: ", end = "")
        printy(f"{current_pos:.2f}")
        print_log(f"MOVE MOTOR: {id}", "Duration (est): ", end = "")
        printy(f"{duration:.2f} s")
        print_log(f"MOVE MOTOR: {id}", "Timeout: ", end = "")
        printy(f"{timeout:.2f} s")
        
    def move_motor_speed(self, speed_rpm):
        # Move Motor Speed in RPM
        speed = int ((speed_rpm * 48.4)/60)
        self.dev.move_motor_speed(self.motor_id, speed)

    def move_motor_pos_step(self, position_rotation, wait_move = True, print_motor_log = False):
        # Move Motor Position in Rotation with Step Profile
        current_pos = self.get_motor_pos()['pos_rotation']
        duration = self.calculate_motion_time(position_rotation-current_pos, self.config.MAX_SPEED_RPM, 100_000)
        timeout = duration+30
        
        if print_motor_log:
            self.print_motor_info(self.motor_id, current_pos, duration, timeout)

        position = int(position_rotation * 48.4)
        self.dev.move_motor_abs_pos(self.motor_id, position)
        if wait_move:
            self.__wait_move_done(timeout)

    def move_motor_pos_trapezoid(self, position_rotation, speed_rpm, acc_rpm, wait_move = True, print_motor_log = False):
        # Move Motor Position in Rotation with Trapezoidal Profile
        current_pos = self.get_motor_pos()['pos_rotation']
        duration = self.calculate_motion_time(position_rotation-current_pos, speed_rpm, acc_rpm)
        timeout = duration+30
        
        if print_motor_log:
            self.print_motor_info(self.motor_id, current_pos, duration, timeout)
        
        position = int(position_rotation * 48.4)
        speed = int ((speed_rpm * 48.4)/60)
        acc = int ((acc_rpm * 48.4)/60)
        self.dev.move_motor_abs_pos_trapezoid(self.motor_id, position, speed, acc)
        if wait_move:
            self.__wait_move_done(timeout)

    def move_motor_open_loop(self, pwm):
        # Move Motor Speed Open Loop
        self.dev.move_motor_open_loop(self.motor_id, pwm)

    def stop_motor(self):
        # Stop the Motor
        self.dev.stop_motor(self.motor_id)
    
    def get_motor_pos(self):
        # Get Motor Position 
        pos = self.dev.get_motor_pos(self.motor_id)
        pos['pos_rotation'] = pos['pos_count']/48.4
        return pos
    
    def get_motor_speed(self):
        # Get Motor Speed
        speed  = self.dev.get_motor_speed(self.motor_id)
        speed['speed_rpm'] = (speed['speed_cps']*60)/48.4
        return speed

    def calculate_motion_time(self, target_distance, speed_rpm, acceleration_rpm_per_s):
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
    
    def move_open_loop(self, pwm, duration=1.0, time_sampling = 1, mode: SystemModel = SystemModel.Nonlinear):
        #############################################################################
        # Directory
        group_name      = "Open_Loop_"
        tag = group_name + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        
        # File Properties
        command_header  = "Commanded_PWM"
        data_header     = "Motor_Speed(RPM)"
        
        # Plot Config
        plot_title          = "Open Loop Response"
        y_label             = "Motor Speed(RPM)"
        data_legend         = "Actual Open Loop Response"
        commanded_legend    = "PWM Input"
        simulation_legend   = "Open Loop Simulation"
        same_unit           = False
        #############################################################################
        
        self.start_log(mask = 18, time_sampling=time_sampling, folder_tag=tag)
        self.move_motor_open_loop(pwm)
        time.sleep(duration)
        self.stop_motor()
        dir_name = self.stop_log()
        
        log_dir = base_url+"/LOG/"+dir_name+"/"
        filename = os.listdir(base_url+"/LOG/"+dir_name)[0]
    
        motor_response = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, command_header, data_header, threshold_percent=2.0)
        
        if motor_response["start_time_s"] is not None:
            simulation_time_list, simulation_speed, simulation_target = self.sim.simulate_open_loop_response(
                                                                    target_pwm  = pwm,
                                                                    u           = motor_response["command_list"],
                                                                    mode        = mode,
                                                                    )
            
            Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                                pid_config                  = None,
                                                simulation_time_list        = simulation_time_list, 
                                                actual_time_list            = motor_response["time_list"],
                                                simulation_data_list        = simulation_speed, 
                                                simulation_commanded_list   = simulation_target,  
                                                actual_data_list            = motor_response["data_list"], 
                                                actual_commanded_list       = motor_response["command_list"],
                                                tag                         = group_name,
                                                plot_title                  = plot_title,
                                                y_label                     = y_label,
                                                actual_data_label           = data_legend,
                                                actual_commanded_label      = commanded_legend,
                                                simulation_data_label       = simulation_legend,
                                                simulation_commanded_label  = commanded_legend,
                                                same_unit                   = same_unit, 
                                                )

    def move_speed(self, speed_rpm, time_sampling = 1, mode: SystemModel = SystemModel.Nonlinear):
        #############################################################################
        # Directory
        group_name      = "Speed_Step_"
        tag = group_name + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        
        # File Properties
        command_header  = "Commanded_Speed(RPM)"
        data_header     = "Motor_Speed(RPM)"
        
        # Plot Config
        plot_title          = "Speed Control Response"
        y_label             = "Motor Speed(RPM)"
        data_legend         = "Actual Motor Response"
        commanded_legend    = "Commanded Speed"
        simulation_legend   = "Speed Simulation"
        same_unit           = True
        #############################################################################
        
        pid_config = self.dev.get_pid_motor_speed(self.motor_id)
    
        kp = pid_config["kp"]
        ki = pid_config["ki"]
        kd = pid_config["kd"]
        i_limit = pid_config["i_limit"]
        
        pid_params = [kp, ki, kd, i_limit]
        
        self.start_log(mask=10, time_sampling=time_sampling, folder_tag=tag)
        self.move_motor_speed(speed_rpm)
        time.sleep(1)
        self.stop_motor()
        dir_name = self.stop_log()
        
        log_dir = base_url+"/LOG/"+dir_name+"/"
        filename = os.listdir(base_url+"/LOG/"+dir_name)[0]
    
        motor_response = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, command_header, data_header, threshold_percent=2.0)
        
        if motor_response["start_time_s"] is not None:
            simulation_time_list, simulation_speed, simulation_target = self.sim.simulate_pid_speed_control(
                                                            pid_params      = pid_params,
                                                            initial_speed   = motor_response["command_list"][0]/self.config.ROTATION_PER_PULSE/60,
                                                            setpoint        = motor_response["command_list"]/self.config.ROTATION_PER_PULSE/60,
                                                            mode            = mode
                                                            )
            
            Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                                pid_config                  = pid_params,
                                                simulation_time_list        = simulation_time_list, 
                                                actual_time_list            = motor_response["time_list"],
                                                simulation_data_list        = simulation_speed, 
                                                simulation_commanded_list   = simulation_target,  
                                                actual_data_list            = motor_response["data_list"], 
                                                actual_commanded_list       = motor_response["command_list"],
                                                tag                         = group_name,
                                                plot_title                  = plot_title,
                                                y_label                     = y_label,
                                                actual_data_label           = data_legend,
                                                actual_commanded_label      = commanded_legend,
                                                simulation_data_label       = simulation_legend,
                                                simulation_commanded_label  = commanded_legend,
                                                same_unit                   = same_unit, 
                                                )

    def move_pos(self, position_rotation, speed_rpm=None, acc_rpm=None, duration=1.5, time_sampling = 1, mode: SystemModel = SystemModel.Nonlinear):
        #############################################################################
        # Directory
        group_name      = "Position_Step_"
        tag = group_name + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        
        # File Properties
        command_header  = "Commanded_Position(rotation)"
        data_header     = "Motor_Position(rotation)"
        
        # Plot Config
        plot_title          = "Position Control Response"
        y_label             = "Motor Position (Rotation)"
        data_legend         = "Actual Motor Response"
        commanded_legend     = "Commanded Position"
        simulation_legend   = "Position Simulation"
        same_unit           = True
        #############################################################################
        
        pid_speed = self.dev.get_pid_motor_speed(self.motor_id)
        pid_speed_config = [pid_speed["kp"], pid_speed["ki"], pid_speed["kd"], pid_speed["i_limit"]]
        
        pid_pos = self.dev.get_pid_motor_pos(self.motor_id)
        pid_pos_config = [pid_pos["kp"], pid_pos["ki"], pid_pos["kd"], pid_pos["i_limit"]] 
        
        self.start_log(mask = 5, time_sampling=time_sampling, folder_tag=tag)
        
        if speed_rpm == None or acc_rpm == None:
            self.move_motor_pos_step(position_rotation, print_motor_log=True)
        else:
            self.move_motor_pos_trapezoid(position_rotation, speed_rpm=speed_rpm, acc_rpm=acc_rpm, print_motor_log=True)
        time.sleep(duration)
        self.stop_motor()
        dir_name = self.stop_log()
        
        log_dir = base_url+"/LOG/"+dir_name+"/"
        filename = os.listdir(base_url+"/LOG/"+dir_name)[0]
    
        motor_response = Tool.FileProcessing.extract_firmware_log_data(log_dir+filename, command_header, data_header, threshold_percent=2.0)
        
        initial_position = motor_response["command_list"][0]
        
        simulation_time_list, simulation_pos, simulation_target, simulation_speed = self.sim.simulate_pid_pos_control(
                                                                pid_pos_params      = pid_pos_config,
                                                                pid_speed_params    = pid_speed_config,
                                                                initial_pos         = initial_position/self.config.ROTATION_PER_PULSE,
                                                                setpoint            = motor_response["command_list"]/self.config.ROTATION_PER_PULSE,
                                                                mode                = mode,
                                                                )
        
        Tool.plotter.create_simulation_plot(log_dir                     = log_dir, 
                                            pid_config                  = pid_pos_config,
                                            simulation_time_list        = simulation_time_list, 
                                            actual_time_list            = motor_response["time_list"],
                                            simulation_data_list        = simulation_pos, 
                                            simulation_commanded_list   = simulation_target,  
                                            actual_data_list            = motor_response["data_list"], 
                                            actual_commanded_list       = None,
                                            tag                         = group_name,
                                            plot_title                  = plot_title,
                                            y_label                     = y_label,
                                            actual_data_label           = data_legend,
                                            actual_commanded_label      = commanded_legend,
                                            simulation_data_label       = simulation_legend,
                                            simulation_commanded_label  = commanded_legend,
                                            same_unit                   = same_unit, 
                                            )                 