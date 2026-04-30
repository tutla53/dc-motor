# Pico Script

import time
import Board.Pico as Board
import BasicFunction.Motor as Motor
import Config.Motor0
import Config.Motor1
import Tool.plotter
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
from scipy.signal import lfilter
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

def get_pwm_identification():
    test_case = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
    
    for pwm in test_case:
        input_pwm = int((pwm* m0.config.MAX_PWM_TICKS)/100)
        open_loop_test(0, input_pwm, time_sampling=1, duration=1)
    
    for pwm in test_case:
        input_pwm = int((pwm* m0.config.MAX_PWM_TICKS)/100)
        open_loop_test(0, -input_pwm, time_sampling=1, duration=1)

def motor_simulation(params, u, dt):
    # Model - First Order ODE
    # G(s)=K/(τs+1)
    
    K, tau = params
    # Convert G(s) to discrete-time coefficients (a and b)
    a = np.exp(-dt / tau)
    b = K * (1 - a)
    
    # y[k] = a*y[k-1] + b*u[k-1]
    # We use lfilter for high-speed simulation of the difference equation
    y_sim = lfilter([0, b], [1, -a], u)
    return y_sim

def objective_function(params, files_data, dt):
    """Calculates the Total Squared Error across all files."""
    total_error = 0
    # Constraint: K and Tau must be positive
    if params[0] <= 0 or params[1] <= 0:
        return 1e10 
        
    for u, y_meas in files_data:
        y_sim = motor_simulation(params, u, dt)
        target = np.max(np.abs(y_meas))
        mse = np.mean((y_meas - y_sim)**2)
        rmse = np.sqrt(mse)
        if target == 0: target = 1 # Avoid division by zero
        
        total_error += rmse/target
        
    return total_error

def system_identification():
    files_data = []
    K_list = []
    tau_list = []
    
    log_dir = base_url+"/LOG/System_Identification/All/"
    all_log_files = os.listdir(log_dir)
    initial_guess = [1, 1]
    
    for filename in all_log_files:
        if not (".csv" in filename):
            continue
        
        df = pd.read_csv(log_dir+filename)
        
        u_data = df["Commanded_PWM"].values
        y_data = df["Motor_Speed(RPM)"].values
        
        u_data = u_data.astype(float)
        y_data = y_data.astype(float) 
        
        files_data.append((u_data, y_data))

        time_stamp = df["Timestamp(ms)"]
        dt_ms = time_stamp[1] - time_stamp[0]
        dt_s = dt_ms/1000

        # result = minimize(
        #     objective_function, 
        #     initial_guess, 
        #     args=(files_data, dt_s), 
        #     method='Nelder-Mead'
        # )
        
        bounds = [(0.01, 0.3), (0.001, 0.1)] # [K, Tau]

        result = differential_evolution(
            objective_function, 
            bounds, 
            args=(files_data, dt_s),
            strategy='best1bin', # Standard strategy
            tol=0.01,
            mutation=(0.5, 1)
        )

        if result.success:
            K, tau = result.x
            K_list.append(K)
            tau_list.append(tau)
            # initial_guess = [K, tau]
        else:
            print("Optimization failed:", result.message)
        
        # Validate Result
        time_axis = df["Timestamp(ms)"]/1000.0
        y_sim = motor_simulation((K, tau), u_data, dt_s)
        
        percent_pwm = (max(abs(u_data))/m0.config.MAX_PWM_TICKS)*100
        
        plt.plot(time_axis, u_data, label='PWM Input')
        plt.plot(time_axis, y_data, label='Experimental Data')
        plt.plot(time_axis, y_sim, '--', label='Model Prediction', color='orange')
        plt.ylabel('Velocity')
        plt.title(f"Validation - {percent_pwm:.2f}")
        plt.legend()
        plt.grid()

        try:
            image_name = "/tuning/graph_"+filename+".jpg"
            plt.savefig(log_dir+image_name, dpi = 300)
            
            _, sep, after = log_dir.partition("/LOG/")
            result = sep + after
            print_log("INFO", "Graph has been saved on: ", end="")
            printg(result+image_name)
            
        except Exception as e:
            printr(f"[ERROR] - {e}" )
        plt.close()         

    for filename in all_log_files:
        if not (".csv" in filename):
            continue
        
        df = pd.read_csv(log_dir+filename)
        
        u_data = df["Commanded_PWM"].values
        y_data = df["Motor_Speed(RPM)"].values
        
        u_data = u_data.astype(float)
        y_data = y_data.astype(float) 
        
        files_data.append((u_data, y_data))

        time_stamp = df["Timestamp(ms)"]
        dt_ms = time_stamp[1] - time_stamp[0]
        dt_s = dt_ms/1000
        
        # Validate Result
        time_axis = df["Timestamp(ms)"]/1000.0
        y_sim = motor_simulation((np.average(K_list), np.average(tau_list)), u_data, dt_s)
        
        percent_pwm = (max(abs(u_data))/m0.config.MAX_PWM_TICKS)*100
        
        plt.plot(time_axis, u_data, label='PWM Input')
        plt.plot(time_axis, y_data, label='Experimental Data')
        plt.plot(time_axis, y_sim, '--', label='Model Prediction', color='orange')
        plt.ylabel('Velocity')
        plt.title(f"Validation - {percent_pwm:.2f}")
        plt.legend()
        plt.grid()

        try:
            image_name = "/prediction/graph_"+filename+"_average.jpg"
            plt.savefig(log_dir+image_name, dpi = 300)
            
            _, sep, after = log_dir.partition("/LOG/")
            result = sep + after
            print_log("INFO", "Graph has been saved on: ", end="")
            printg(result+image_name)
            
        except Exception as e:
            printr(f"[ERROR] - {e}" )
        plt.close()             
        
    for k in K_list:
        print(k)
    printg("Average:", np.average(K_list))
    print()

    for tau in tau_list:
        print(tau)
    printg("Average:", np.average(tau_list))
    print()

# Average K: 0.23570627057644225
# Average tau: 0.04888222683160024

# Average K: 0.24126270336163416
# Average tau: 0.04858428774840934