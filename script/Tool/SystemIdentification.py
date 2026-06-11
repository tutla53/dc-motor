import os
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import Tool.FileProcessing
import Tool.MotorSim
import Tool.plotter

from tqdm import tqdm
from scipy.optimize import differential_evolution
from scipy.optimize import least_squares
from base_url import *
from Tool.visualize import *

class MotorOptimization:
    def __init__(self, configfile):
        lower_bounds    = [0.01, 0.01, 0.00]
        upper_bounds    = [0.50, 0.10, 0.05]
        
        self.config         = configfile
        self.dt_s           = self.config.DT_S 
        self.motor          = Tool.MotorSim.SimulateMotor(configfile)
        self.optimize       = OptimizationMethod(lower_bounds, upper_bounds)
        self.__assets_dir   = base_url[:-7] + "/assets/01_System_Identification/open-loop-responses/"
    
    def run_system_identification(self, method="differential_evolution"):
        Params_List     = []
        success_count   = 0
        failed_count    = 0
        
        all_log_files = os.listdir(self.__assets_dir)
        
        for filename in tqdm(all_log_files, desc="System Identification", unit="file"):
            if ".csv" not in filename: continue
            
            df = pd.read_csv(self.__assets_dir + filename)
            u_data = df["Commanded_PWM"].values
            y_data = df["Motor_Speed(RPM)"].values / (60.0 * self.config.ROTATION_PER_PULSE)
            target = np.max(np.abs(u_data)) if u_data[0] < u_data[-10] else -np.max(np.abs(u_data))
            
            data = (u_data.astype(float), y_data.astype(float))
            self.dt_s = (df["Timestamp(ms)"][1] - df["Timestamp(ms)"][0]) / 1000 

            result = self.optimize.calculate(data, self.dt_s, method = method)

            if result.success:
                K, tau, L = result.x
                success_count += 1
                Params_List.append([target, K, tau, L])
            else:
                failed_count += 1
        
        print_log("INFO", f"{success_count}/{success_count+failed_count} Success")
        
        # Save the Result to CSV        
        Sorted_Param_List = sorted(Params_List, key=lambda x: (x[0]))
        tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        log_dir = base_url+"/LOG/System_Identification"+tag+"/"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filename = log_dir + "system_identification_"+method+"_"+tag+".csv"
        columns = ["PWM", "K", "tau", "L"]
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            writer.writerow(columns)
            for row in Sorted_Param_List:
                writer.writerow(row)
        
        Tool.plotter.create_system_identification_graph(filename, self.config, log_dir, tag)
        
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

class OptimizationMethod:
    def __init__(self, lower_bounds, upper_bounds):
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        
    def calculate(self, data, dt_s, method="least_square"):
        if method == "least_square":
            return self.__run_least_square(data, dt_s)
        elif method == "differential_evolution":
            return self.__run_differential_evolution(data, dt_s)
        else:
            return None
    
    def __open_loop_response(self, params, u: list, dt):
        '''
            Input u: PWM (Ticks)
            Output y: Motor Speed (Pulse per Second)
        '''
        
        K, tau, L = params
        
        # Simulation Variable
        N   = len(u)        # Number of Data
        d   = int(L / dt)   # Time Delay
        y   = [0.0] * N     # Output: Motor Speed (Pulse per Second)
        
        # Motor Parameters
        ALPHA   = np.exp(-dt / tau)
        BETA    = K * (1 - ALPHA)
        
        for k in range(N):
            if (k - d - 1) < 0:
                continue
            # Difference Equation: Update Speed
            y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]
            
        return y
    
    def __run_least_square(self, data, dt_s):
                
        best_res = None
        best_cost = np.inf
        
        max_data = int(self.upper_bounds[2] * 1000)

        for L_candidate in range(0, max_data+1):
            
            lower_bounds    = self.lower_bounds
            upper_bounds    = self.upper_bounds
            initial_guess   = [0.25, 0.05, float(L_candidate) * dt_s]

            result = least_squares(
                self.__least_square_objective_function, 
                x0=initial_guess, 
                bounds=(lower_bounds, upper_bounds), 
                args=(data, dt_s),
                method='trf',
                ftol=1e-3,
                xtol=1e-3
            )
            
            if result.success and result.cost < best_cost:
                best_cost   = result.cost
                best_res    = result
        
        return best_res
    
    def __least_square_objective_function(self, params, data, dt):

        u, y_meas = data
        
        y_sim = self.__open_loop_response(params, u.tolist(), dt)
        
        error = y_meas - y_sim
        
        target = np.max(np.abs(y_meas))
        if target == 0: 
            target = 1.0
            
        return error / target
    
    def __run_differential_evolution(self, data, dt_s):
        bounds =[   (self.lower_bounds[0], self.upper_bounds[0]),   # K
                    (self.lower_bounds[1], self.upper_bounds[1]),   # tau
                    (self.lower_bounds[2], self.upper_bounds[2])    # L
                ] 

        result = differential_evolution(
            self.__differential_evolution_objective_function, 
            bounds, 
            args=(data, dt_s),
            strategy='best1bin',
            tol=0.01,
            mutation=(0.5, 1),
            polish=True
        )
        
        return result
    
    def __differential_evolution_objective_function(self, params, data, dt):
        # Method: Root Mean Square Error (RMSE)
        
        K, tau, L = params
        
        # Filter Invalid Value
        if K <= 0 or tau <= 0 or L < 0:
            return 1e10 
        
        u, y_meas = data
        y_sim = self.__open_loop_response(params, u.tolist(), dt)
        
        error = y_meas - y_sim
        mse = np.mean(error**2)
        rmse = np.sqrt(mse)
        
        target = np.max(np.abs(y_meas))
        if target == 0: target = 1 
        
        total_error = rmse / target
            
        return total_error    