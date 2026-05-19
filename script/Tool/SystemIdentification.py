import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import BasicFunction.Motor as Motor
import Tool.FileProcessing

from tqdm import tqdm
from scipy.optimize import differential_evolution
from scipy.signal import lfilter
from base_url import *
from Tool.visualize import *

class MotorSim:
    def __init__(self, configfile):
        self.config = configfile
    
    def simulate_open_loop_response(self, params, u, dt):
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
        
        K, tau, L = params
        
        delay_samples = int(round(L / dt))
        
        # Shift the input signal u by 'delay_samples'
        if delay_samples > 0:
            u_delayed = np.zeros_like(u)
            u_delayed[delay_samples:] = u[:-delay_samples]
        else:
            u_delayed = u

        # Discrete-time coefficients
        a = np.exp(-dt / tau)
        b = K * (1 - a)
        
        # Simulate the first-order response
        y_sim = lfilter([0, b], [1, -a], u_delayed)
        return y_sim

    def simulate_pid_speed_control(self, pid_params, target, start_time, duration):
        dt_s = self.config.DT_S
        steps = int(duration / dt_s)
        t = np.linspace(0, duration, steps)
        
        current_speed   = 0.0
        integral        = 0.0
        prev_error      = 0.0
        
        speed_history       = []
        setpoint_history    = []
        pwm_buffer          = [0.0] * self.config.DELAY_STEPS # Hardware Latency Buffer
        
        # PID Config
        kp, ki, kd, i_limit = pid_params
            
        # System Properties
        max_pwm = self.config.MAX_PWM_TICKS
        alpha = np.exp(-dt_s / self.config.TAU_S)
        beta = self.config.K * (1 - alpha)
        
        for i in range(steps):
            setpoint = target if t[i] >= start_time else 0.0
            error = setpoint - current_speed
            
            # --- RUST IMPLEMENTATION LOGIC ---
            integral += error
            integral = np.clip(integral, -i_limit, i_limit)
            
            # let derivative = error - self.prev_error;
            derivative = error - prev_error
            prev_error = error
            
            # let sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);
            pwm_out = (kp * error) + (ki * integral) + (kd * derivative)
            pwm_out = np.clip(pwm_out, -max_pwm, max_pwm)
            
            # Delay Handler
            pwm_buffer.append(pwm_out)
            delayed_pwm = pwm_buffer.pop(0)
            
            # --- Physical Response ---
            current_speed = (alpha * current_speed) + (beta * delayed_pwm)
            
            speed_history.append(current_speed)
            setpoint_history.append(setpoint)
            
        return t, np.array(speed_history), np.array(setpoint_history)

class MotorOptimization:
    def __init__(self, configfile):
        self.config = configfile
        self.motor = MotorSim(configfile)
        self.__assets_dir = base_url + "/assets/open-loop-responses/"
    
    def run_system_identification(self):
        files_data = []
        K_list = []
        tau_list = []
        L_list = []
        success_count = 0
        failed_count = 0
        
        all_log_files = os.listdir(self.__assets_dir)
        
        dt_s = 0.005 # Default Time Sampling
        
        for filename in tqdm(all_log_files, desc="System Identification", unit="file"):
            if ".csv" not in filename: continue
            
            df = pd.read_csv(self.__assets_dir + filename)
            u_data = df["Commanded_PWM"].values
            y_data = df["Motor_Speed(RPM)"].values
            target = round((np.max(np.abs(u_data))/self.config.MAX_PWM_TICKS)*100)
            
            files_data.append((u_data.astype(float), y_data.astype(float)))
            
            dt_s = (df["Timestamp(ms)"][1] - df["Timestamp(ms)"][0]) / 1000 # Update dt based on the actual log file

            bounds =    [(0.01, 0.5),    # K
                        (0.01, 0.1),   # tau
                        (0.0, 0.05)]   # L

            result = differential_evolution(
                self.__system_identification_objective_function, 
                bounds, 
                args=(files_data, dt_s),
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
    
    def __system_identification_objective_function(self, params, files_data, dt):
        # Method: Root Mean Square Error (RMSE)
        
        K, tau, L = params
        
        # Filter Invalid Value
        if K <= 0 or tau <= 0 or L < 0:
            return 1e10 
            
        total_error = 0
        for u, y_meas in files_data:
            y_sim = self.motor.simulate_open_loop_response(params, u, dt)
            
            error = y_meas - y_sim
            mse = np.mean(error**2)
            rmse = np.sqrt(mse)
            
            target = np.max(np.abs(y_meas))
            if target == 0: target = 1 
            
            total_error = rmse / target
            
        return total_error

    def plot_rmse(self, positive_axis_only=True):
        K, tau, L = self.run_system_identification()
        
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
                percent_pwm = round((max(u_data)/self.config.MAX_PWM_TICKS)*100)
            else:
                if positive_axis_only:
                    continue
                else:
                    percent_pwm = round((min(u_data)/self.config.MAX_PWM_TICKS)*100)
            
            param = (K, tau, L)
            y_sim = self.motor.simulate_open_loop_response(param, u_data, dt_s)
            
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

    def run_optimize_pid_speed(self):
        # TARGET_SPEED = [300, 400, 500, 600, 700, 800, 900, 1000, 1100]
        dspeed = 10
        TARGET_SPEED = range(300, 1100+dspeed, dspeed)
        START_TIME = 0.4
        DURATION = 1.5
        
        KP = []
        KI = []
        KD = []

        for target_speed in tqdm(TARGET_SPEED, desc="Optimize PID Speed", unit="target"):
            # Search bounds for [Kp, Ki, Kd, i_limit]
            bounds = [
                (0.0, 10.0),    # Kp
                (0.0, 20.0),    # Ki
                (0.0, 10.0),    # Kd
                (self.config.MAX_PWM_TICKS, self.config.MAX_PWM_TICKS)   # i_limit
            ]
            
            result = differential_evolution(
                self.__pid_speed_objective_function,
                bounds=bounds,
                args=(target_speed, START_TIME, DURATION),
                strategy='best1bin',
                maxiter=100,
                popsize=15,
                tol=0.01,
                seed=42,
                disp=False
            )
            
            best_kp, best_ki, best_kd, best_i_limit = result.x
            
            KP.append(best_kp)
            KI.append(best_ki)
            KD.append(best_kd)
        
        final_Kp = np.median(KP)
        final_Ki = np.median(KI)
        final_Kd = np.median(KD)
        i_limit = self.config.MAX_PWM_TICKS
        
        print_log("INFO", f"Median Kp: {final_Kp}")
        print_log("INFO", f"Median Ki: {final_Ki}")
        print_log("INFO", f"Median Kd: {final_Kd}")
        print_log("INFO", f"Median i_limit: {i_limit}")    
        
        return final_Kp, final_Ki, final_Kd, i_limit
    
    def __pid_speed_objective_function(self, pid_params, target, start_time, duration):
        # Method: Root Mean Square Error (RMSE)
        _, speed, setpoint = self.motor.simulate_pid_speed_control(pid_params, target, start_time, duration)
        
        error = setpoint - speed
        rmse = np.sqrt(np.mean(error**2))
        
        if np.isnan(rmse):
            return 1e10
            
        return rmse
