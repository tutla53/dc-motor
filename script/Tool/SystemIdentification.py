import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import Tool.FileProcessing

from tqdm import tqdm
from bisect import bisect_left
from scipy.optimize import differential_evolution
from scipy.signal import lfilter
from base_url import *
from Tool.visualize import *
from enum import Enum

class SystemModel(Enum):
    Linear      = 0
    Nonlinear   = 1
    
class PIDControl():
    def __init__(self, kp, ki, kd, i_limit, max_threshold):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.i_limit        = i_limit
        self.integral       = 0
        self.prev_error     = 0
        self.max_threshold  = max_threshold

    def reset(self):
        self.integral   = 0
        self.prev_error = 0
        
    def compute(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
        
        derivative = error - self.prev_error
        self.prev_error = error
        
        pwm_out = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        pwm_out = np.clip(pwm_out, -self.max_threshold, self.max_threshold)
        return pwm_out

class MotorSim:
    def __init__(self, configfile):
        self.config = configfile
        
        # System Parameter
        self.K       = self.config.K                # Linear Gain
        self.tau     = self.config.TAU_S            # Time Constant
        self.L       = self.config.DELAY_STEPS      # Time Delay
        self.DT_S    = self.config.DT_S             # Time Sampling
        
        # Nonlinear Motor Gain
        self.K_LIST     =  self.config.K_LIST       # Nonlinear Gain Based on PWM Input
        self.PWM_LIST   =  self.config.PWM_LIST
        
        # System Config
        self.RPM_PER_PPS            = (60.0*self.config.ROTATION_PER_PULSE)
        self.PPS_PER_RPM            = 1 / self.RPM_PER_PPS
        self.ROTATION_PER_PULSE     = self.config.ROTATION_PER_PULSE
        self.PULSE_PER_ROTATION     = 1 / self.ROTATION_PER_PULSE
        self.MAX_PWM                = self.config.MAX_PWM_TICKS
        self.MAX_SPEED_RPS          = self.config.MAX_SPEED_RPM_CONTROL * self.PPS_PER_RPM  # 1400 RPM for control calculation
        self.MAX_SPEED_RPM          = self.config.MAX_SPEED_RPM
        
        # Linear Discrete Model System
        self.ALPHA      = np.exp(-self.DT_S / self.tau)
        self.BETA       = self.K * (1 - self.ALPHA)
    
    def __calculate_nonlinear_K(self, input_pwm):    
        
        if input_pwm <= self.PWM_LIST[0]:
            return self.K_LIST[0]
        if input_pwm >= self.PWM_LIST[-1]:
            return self.K_LIST[-1]
        
        idx = bisect_left(self.PWM_LIST, input_pwm)
        
        p0, p1 = self.PWM_LIST[idx-1], self.PWM_LIST[idx]
        k0, k1 = self.K_LIST[idx-1], self.K_LIST[idx]
        weight = (input_pwm - p0) / (p1 - p0)
        K = k0 + weight * (k1 - k0)
        
        return K

    def __update_motor_speed(self, current_speed_pps, input_pwm, mode: SystemModel = SystemModel.Nonlinear):
        '''
            First Order System with Delay
            
            Input:
                pwm (ticks)
            Output:
                speed (pulse per seconds)
            
            Transfer Function
                        K * e^(-L * s)
                G(s) = ----------------
                        τ * s + 1
            
            Where:
                K   = Gain          ((pulse per seconds)/ ticks)
                τ   = Time Constant (seconds)
                L   = Delay Time    (seconds)
            
            Discrete Form with sampling time dt: 
                y[k] = a·y[k-1] + b·u[k-d-1]
                
                with:
                    a = e^(-dt / τ)
                    b = K * (1-a)
                    d = time_delay
        '''
        
        if mode == SystemModel.Linear:
            BETA = self.BETA
        elif mode == SystemModel.Nonlinear:
            BETA = self.__calculate_nonlinear_K(input_pwm) * (1 - self.ALPHA)
        
        new_speed_pps = (self.ALPHA * current_speed_pps) + (BETA * input_pwm)
        
        return new_speed_pps
    
    def simulate_pid_speed_control(self, pid_params, target_rpm, start_time, duration):
        """
            Target unit                 --> RPM
            PID speed calculation unit  --> pulse per second
            Speed output unit           --> RPM
        """
        
        # Simulation Variable
        target_rpm = np.clip(target_rpm, -self.MAX_SPEED_RPM, self.MAX_SPEED_RPM)
            
        target_pps  = target_rpm * self.PPS_PER_RPM
        steps       = int(duration / self.DT_S)
        time_axis   = np.linspace(0, duration, steps)
        
        current_speed_pps   = 0.0   # pulse per seconds
        speed_list_rpm      = []    # RPM
        setpoint_list_rpm   = []    # RPM
        pwm_buffer          = [0.0] * self.L # Hardware Latency Buffer
        
        # PID Speed Handler
        kp, ki, kd, i_limit = pid_params        
        speed_control = PIDControl(kp, ki, kd, i_limit, self.MAX_PWM)
        
        for i in range(steps):
            setpoint_pps = target_pps if time_axis[i] >= start_time else 0.0
            speed_error = setpoint_pps - current_speed_pps
            
            pwm_out = speed_control.compute(speed_error)
            
            # Delay Handler
            pwm_buffer.append(pwm_out)
            delayed_pwm = pwm_buffer.pop(0)
            
            # Physical Response
            current_speed_pps = self.__update_motor_speed(current_speed_pps, delayed_pwm, SystemModel.Nonlinear)
            
            speed_list_rpm.append(current_speed_pps * self.RPM_PER_PPS)
            setpoint_list_rpm.append(setpoint_pps * self.RPM_PER_PPS)
            
        return time_axis, np.array(speed_list_rpm), np.array(setpoint_list_rpm)

    def simulate_pid_pos_control(self, pid_pos_params, pid_speed_params, target_rotation, start_time, duration):
        """
            Target unit                 --> rotation
            PID pos calculation unit    --> pulse
            PID speed calculation unit  --> pulse per seconds
            Speed output unit           --> RPM
            Position output unit        --> rotation
        """
        
        # Simulation Variable
        target_pulse    = target_rotation * self.PULSE_PER_ROTATION
        steps           = int(duration / self.DT_S)
        time_axis       = np.linspace(0, duration, steps)
        
        current_speed_pps       = 0.0   # pulse per second
        prev_speed_pps          = 0.0   # pulse per second
        current_pos_pulse       = 0.0   # pulse 
        pos_list_rotation       = []    # rotation
        speed_list_rpm          = []    # rpm
        setpoint_list_rotation  = []    # rotation
        pwm_buffer              = [0.0] * self.L    # Hardware Latency Buffer
        
        # PID Handler
        kp_speed, ki_speed, kd_speed, i_limit_speed = pid_speed_params
        kp_pos, ki_pos, kd_pos, i_limit_pos = pid_pos_params

        speed_control       = PIDControl(kp_speed, ki_speed, kd_speed, i_limit_speed, self.MAX_PWM)
        position_control    = PIDControl(kp_pos, ki_pos, kd_pos, i_limit_pos, self.MAX_SPEED_RPS)
        
        for i in range(steps):
            setpoint_pulse = target_pulse if time_axis[i] >= start_time else 0.0
            
            pos_error = setpoint_pulse - current_pos_pulse          # pulse
            target_speed_pps = position_control.compute(pos_error)  # pulse per second
            
            speed_error = target_speed_pps - current_speed_pps      # pulse per second
            pwm_out = speed_control.compute(speed_error)            # ticks
            
            # Delay Handler
            pwm_buffer.append(pwm_out)
            delayed_pwm = pwm_buffer.pop(0)
            
            # --- Physical Response ---
            prev_speed_pps = current_speed_pps
            current_speed_pps = self.__update_motor_speed(current_speed_pps, delayed_pwm, SystemModel.Nonlinear)
            
            current_pos_pulse += ((prev_speed_pps + current_speed_pps)/2.0) * self.DT_S
            
            pos_list_rotation.append(current_pos_pulse * self.ROTATION_PER_PULSE)    # rotation
            speed_list_rpm.append(current_speed_pps * self.RPM_PER_PPS)              # RPM
            setpoint_list_rotation.append(setpoint_pulse * self.ROTATION_PER_PULSE)  # rotation
            
        return time_axis, np.array(pos_list_rotation), np.array(setpoint_list_rotation), np.array(speed_list_rpm)
    
    def simulate_open_loop_response(self, target_pwm, start_time, duration):
        """
            Target unit         --> pwm_ticks
            Speed output unit   --> RPM
        """
        
        # Simulation Variable
        steps       = int(duration / self.DT_S)
        time_axis   = np.linspace(0, duration, steps)
        
        current_speed_pps   = 0.0   # pulse per seconds
        speed_list_rpm      = []    # RPM
        setpoint_list_ticks = []    # RPM
        pwm_buffer          = [0.0] * self.L # Hardware Latency Buffer
        
        target_pwm = np.clip(target_pwm, -self.MAX_PWM, self.MAX_PWM)
        
        for i in range(steps):
            setpoint_pwm_ticks = target_pwm if time_axis[i] >= start_time else 0.0
            
            # Delay Handler
            pwm_buffer.append(setpoint_pwm_ticks)
            delayed_pwm = pwm_buffer.pop(0)
            
            # Physical Response
            current_speed_pps = self.__update_motor_speed(current_speed_pps, delayed_pwm, SystemModel.Nonlinear)
            
            speed_list_rpm.append(current_speed_pps * self.RPM_PER_PPS)
            setpoint_list_ticks.append(setpoint_pwm_ticks)
            
        return time_axis, np.array(speed_list_rpm), np.array(setpoint_list_ticks)

class MotorOptimization:
    def __init__(self, configfile):
        self.config = configfile
        self.motor = MotorSim(configfile)
        self.__assets_dir = base_url + "/assets/open-loop-responses/"
    
    def __open_loop_response(self, params, u, dt):        
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
    
    def run_system_identification(self):
        files_data      = []
        K_list          = []
        tau_list        = []
        L_list          = []
        success_count   = 0
        failed_count    = 0
        
        all_log_files = os.listdir(self.__assets_dir)
        
        dt_s = 0.005 # Default Time Sampling
        
        for filename in tqdm(all_log_files, desc="System Identification", unit="file"):
            if ".csv" not in filename: continue
            
            df = pd.read_csv(self.__assets_dir + filename)
            u_data = df["Commanded_PWM"].values
            y_data = df["Motor_Speed(RPM)"].values/(60.0*self.config.ROTATION_PER_PULSE)
            target = np.max(np.abs(u_data)) if u_data[0] > u_data[-1] else -np.max(np.abs(u_data))
            
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
            y_sim = self.__open_loop_response(params, u, dt)
            
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
            
            motor_response = Tool.FileProcessing.extract_firmware_log_data(assets_dir+filename, "Commanded_PWM", "Motor_Speed(RPM)")
            
            u_data = motor_response["command_list"]
            y_data = motor_response["data_list"]
            
            if abs(max(u_data)) > abs(min(u_data)):
                percent_pwm = round((max(u_data)/self.config.MAX_PWM_TICKS)*100)
            else:
                if positive_axis_only:
                    continue
                else:
                    percent_pwm = round((min(u_data)/self.config.MAX_PWM_TICKS)*100)
            
            param = (K, tau, L)
            y_sim = self.__open_loop_response(param, u_data, motor_response["dt_s"])
            
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

    def run_tune_pid_speed(self):
        TARGET_SPEED                = [-1200, -1100, -1000, -900, -800, -700, -600, -500, 500, 600, 700, 800, 900, 1000, 1100, 1200]
        START_TIME                  = 0.4
        DURATION                    = 1.0
        MAXIMUM_OVERSHOOT_PERCENT   = 7.0
        MAXIMUM_SETTLING_TIME_S     = 0.3
        
        KP = []
        KI = []
        KD = []
        I_LIMIT = []
        
        success_count   = 0
        failed_count    = 0

        for target_speed in tqdm(TARGET_SPEED, desc="Optimize PID Speed", unit="target"):
            # Search bounds for [Kp, Ki, Kd, i_limit]
            bounds = [
                (0.0, 5.0),    # Kp
                (0.8, 1.0),    # Ki
                (0.0, 20.0),    # Kd
                (self.config.MAX_PWM_TICKS, self.config.MAX_PWM_TICKS)   # i_limit
            ]
            
            result = differential_evolution(
                self.__pid_speed_objective_function,
                bounds=bounds,
                args=(target_speed, START_TIME, DURATION, MAXIMUM_OVERSHOOT_PERCENT, MAXIMUM_SETTLING_TIME_S),
                strategy='best1bin',
                maxiter=100,
                popsize=15,
                tol=0.01,
                seed=42,
                polish=True,
                disp=False
            )
            
            best_kp, best_ki, best_kd, best_i_limit = result.x
            
            if result.success:
                best_kp, best_ki, best_kd, best_i_limit = result.x
                success_count += 1
                KP.append(best_kp)
                KI.append(best_ki)
                KD.append(best_kd)
                I_LIMIT.append(best_i_limit)
            else:
                failed_count +=1
        
        print_log("INFO", f"{success_count}/{success_count+failed_count} Success")
        final_Kp    = np.median(KP)
        final_Ki    = np.median(KI)
        final_Kd    = np.median(KD)
        i_limit     = np.median(I_LIMIT)
        
        print_log("INFO", f"Median Kp: {final_Kp} {np.mean(KP)}")
        print_log("INFO", f"Median Ki: {final_Ki} {np.mean(KI)}")
        print_log("INFO", f"Median Kd: {final_Kd} {np.mean(KD)}")
        print_log("INFO", f"Median i_limit: {i_limit} {np.mean(I_LIMIT)}")      
        
        return final_Kp, final_Ki, final_Kd, i_limit
    
    def __pid_speed_objective_function(self, pid_params, target, start_time, duration, MAXIMUM_OVERSHOOT_PERCENT, MAXIMUM_SETTLING_TIME_S):
        
        time_axis, speed, setpoint = self.motor.simulate_pid_speed_control(pid_params, target, start_time, duration)
        
        # Check for Overshoot
        overshoot = (abs((abs(target) - max(abs(speed)))/target)) * 100
        
        _ , settling_time = Tool.FileProcessing.calculate_settling_time(time_axis, speed, setpoint, threshold_percent=0.001)
        
        # Check overshoot and settling time
        weight = 1
        
        if (settling_time == None) or (settling_time > MAXIMUM_SETTLING_TIME_S):
            weight *= 100
        if (overshoot > MAXIMUM_OVERSHOOT_PERCENT):
            weight *= 100
            
        
        # Method: Root Mean Square Error (RMSE)
        error = setpoint - speed
        rmse = np.sqrt(np.mean(error**2))
        
        if np.isnan(rmse):
            return 1e10
            
        return rmse * weight