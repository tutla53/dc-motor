import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import BasicFunction.Motor as Motor
import Tool.FileProcessing

from tqdm import tqdm
from scipy.optimize import differential_evolution
from base_url import *
from Tool.visualize import *

# Motor Model
'''
    First Order System with Delay
    Transfer Function
                K * e^(-L * s)
        G(s) = ----------------
                τ * s + 1
    
    Where:
        K   = Gain
        τ   = Time Constant (seconds)
        L   = Delay Time (seconds)
    
    Discrete Form: 
            y[k] = a·y[k-1] + b·u[k-d-1]
'''

def objective_function(params, files_data, motor:Motor.MoveMotor, dt):
    # Method: Root Mean Square Error (RMSE)
    
    K, tau, L = params
    
    # Filter Invalid Value
    if K <= 0 or tau <= 0 or L < 0:
        return 1e10 
        
    total_error = 0
    for u, y_meas in files_data:
        y_sim = motor.simulate_open_loop_response(params, u, dt)
        
        error = y_meas - y_sim
        mse = np.mean(error**2)
        rmse = np.sqrt(mse)
        
        target = np.max(np.abs(y_meas))
        if target == 0: target = 1 
        
        total_error = rmse / target
        
    return total_error

def run_identification(motor: Motor.MoveMotor):
    files_data = []
    K_list = []
    tau_list = []
    L_list = []
    success_count = 0
    failed_count = 0
    
    assets_dir = base_url + "/assets/open-loop-responses/"
    all_log_files = os.listdir(assets_dir)
    
    dt_s = 0.005 # Default Time Sampling
    
    for filename in tqdm(all_log_files, desc="Processing Logs", unit="file"):
        if ".csv" not in filename: continue
        
        df = pd.read_csv(assets_dir + filename)
        u_data = df["Commanded_PWM"].values
        y_data = df["Motor_Speed(RPM)"].values
        target = round((np.max(np.abs(u_data))/motor.config.MAX_PWM_TICKS)*100)
        
        files_data.append((u_data.astype(float), y_data.astype(float)))
        
        dt_s = (df["Timestamp(ms)"][1] - df["Timestamp(ms)"][0]) / 1000 # Update dt based on the actual log file

        bounds =    [(0.01, 0.5),    # K
                    (0.01, 0.1),   # tau
                    (0.0, 0.05)]   # L

        result = differential_evolution(
            objective_function, 
            bounds, 
            args=(files_data, motor, dt_s),
            strategy='best1bin',
            tol=0.01,
            mutation=(0.5, 1),
            polish=True
        )

        if result.success:
            K, tau, L = result.x
            success_count += 1
            K_list.append((K, target))
            tau_list.append((tau, target))
            L_list.append((L, target))

        else:
            failed_count +=1
    
    print_log("INFO", f"{success_count}/{success_count+failed_count} Success")
    
    sorted_K = sorted(K_list, key=lambda x: (x[0], x[1]))
    sorted_tau = sorted(tau_list, key=lambda x: (x[0], x[1]))
    sorted_L = sorted(L_list, key=lambda x: (x[0], x[1]))
    
    K_list, _ = map(list, zip(*sorted_K))
    tau_list, _ = map(list, zip(*sorted_tau))
    L_list, _ = map(list, zip(*sorted_L))
    
    final_K = np.median(K_list)
    final_tau = np.median(tau_list)
    final_L = np.median(L_list)
    
    print_log("INFO", f"Median K: {final_K}")
    print_log("INFO", f"Median tau: {final_tau}")
    print_log("INFO", f"Median L: {final_L}")
    
    return final_K, final_tau, final_L


def plot_rmse(motor, positive_axis_only=True):
    K, tau, L = run_identification(motor)
    
    assets_dir = base_url + "/assets/open-loop-responses/"
    all_log_files = os.listdir(assets_dir)
    
    rmse_list = []
    pwm_list = []
    
    for filename in all_log_files:
        if ".csv" not in filename: continue
        
        extracted_data = Tool.FileProcessing.extract_firmware_log_data(assets_dir+filename, "Commanded_PWM", "Motor_Speed(RPM)")
        target, _, _, dt_s, actual_commanded, actual_speed, _ = extracted_data
        
        u_data = actual_commanded.values
        y_data = actual_speed.values
        
        if abs(max(u_data)) > abs(min(u_data)):
            percent_pwm = round((max(u_data)/motor.config.MAX_PWM_TICKS)*100)
        else:
            if positive_axis_only:
                continue
            else:
                percent_pwm = round((min(u_data)/motor.config.MAX_PWM_TICKS)*100)
        
        param = (K, tau, L)
        y_sim = motor.simulate_open_loop_response(param, u_data, dt_s)
        
        mse = np.mean((y_sim - y_data)**2)
        rmse = np.sqrt(mse)
        
        target = np.max(np.abs(y_data))
        if target == 0: target = 1 
        
        total_error = (rmse / target)*100
        rmse_list.append(total_error)
        pwm_list.append(percent_pwm)
                    
    sorted_pairs = sorted(zip(pwm_list, rmse_list))
    pwm_list, rmse_list = map(list, zip(*sorted_pairs))
    
    plt.figure(figsize=(16, 4))
    plt.plot(pwm_list, rmse_list, label='PWM Input', marker='*')
    plt.xlabel('Percent PWM')
    plt.ylabel('RMSE (%)')
    plt.title(f"Motor RMSE Value")
    plt.ylim([0, 20])
    plt.tight_layout()
    plt.legend()
    plt.grid()

    try:
        log_dir = base_url+"/LOG/"
        image_name = "System_Identification_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))+".jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        print(f"[ERROR] - {e}" )
    plt.close()