import csv
import time
from pathlib import Path
import numpy as np
import pandas as pd
import Tool.plotter

from tqdm import tqdm
from scipy.optimize import differential_evolution
from scipy.optimize import least_squares
from base_url import base_url
from Tool.visualize import print_log

METHODS = ("differential_evolution", "least_square")


def read_identification_data(filename, rotation_per_pulse):
    df = pd.read_csv(filename)
    required = ["Timestamp(ms)", "Commanded_PWM", "Motor_Speed(RPM)"]
    missing = set(required).difference(df.columns)
    if missing:
        raise ValueError(f"Missing columns: {sorted(missing)}")
    if len(df) < 10:
        raise ValueError("At least 10 samples are required")
    values = df[required].to_numpy(dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("Input data contains non-finite values")
    intervals = np.diff(values[:, 0]) / 1000.0
    dt_s = float(np.median(intervals))
    if np.any(intervals <= 0):
        raise ValueError("Timestamps must be strictly increasing")
    if not np.allclose(intervals, dt_s, rtol=1e-6, atol=1e-9):
        raise ValueError("Timestamps must be uniformly spaced")
    if not np.isfinite(rotation_per_pulse) or rotation_per_pulse <= 0:
        raise ValueError("ROTATION_PER_PULSE must be finite and positive")
    u_data = values[:, 1]
    y_data = values[:, 2] / (60.0 * rotation_per_pulse)
    target = np.max(np.abs(u_data)) if u_data[0] < u_data[-10] else -np.max(np.abs(u_data))
    return (u_data, y_data), dt_s, target

class MotorOptimization:
    def __init__(self, configfile, asset_dir):
        lower_bounds    = [0.01, 0.01, 0.00]
        upper_bounds    = [0.50, 0.10, 0.05]
        
        self.config         = configfile
        self.optimize       = OptimizationMethod(lower_bounds, upper_bounds)
        self.__assets_dir   = Path(asset_dir)
    
    def run_system_identification(self, method="differential_evolution"):
        if method not in METHODS:
            raise ValueError(f"Unsupported identification method: {method}")
        Params_List     = []
        success_count   = 0
        failed_count    = 0
        
        all_log_files = sorted(
            path for path in self.__assets_dir.iterdir()
            if path.is_file() and path.suffix.lower() == ".csv"
        )
        
        for filename in tqdm(all_log_files, desc="System Identification", unit="file"):
            try:
                data, dt_s, target = read_identification_data(filename, self.config.ROTATION_PER_PULSE)
                result = self.optimize.calculate(data, dt_s, method=method)
                if result is None or not result.success:
                    raise ValueError("Optimizer did not converge")
                if np.shape(result.x) != (3,) or not np.isfinite(result.x).all():
                    raise ValueError("Optimizer returned invalid parameters")
            except (OSError, ValueError, RuntimeError, FloatingPointError) as error:
                failed_count += 1
                print_log("WARN", f"{filename.name}: {error}")
                continue
            else:
                K, tau, L = result.x
                success_count += 1
                Params_List.append([target, K, tau, L])
        
        print_log("INFO", f"{success_count}/{success_count+failed_count} Success")
        if not Params_List:
            print_log("WARN", "No successful fits; no results or graphs were saved.")
            return
        
        # Save the Result to CSV        
        Sorted_Param_List = sorted(Params_List, key=lambda x: (x[0]))
        tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        log_dir = base_url+"/LOG/System_Identification"+tag+"/"
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        
        filename = log_dir + "system_identification_"+method+"_"+tag+".csv"
        columns = ["PWM", "K", "tau", "L"]
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            writer.writerow(columns)
            for row in Sorted_Param_List:
                writer.writerow(row)
        
        Tool.plotter.create_system_identification_graph(filename, self.config, log_dir, tag)

class OptimizationMethod:
    def __init__(self, lower_bounds, upper_bounds):
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        
    def calculate(self, data, dt_s, method="least_square"):
        if not np.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("Sample interval must be finite and positive")
        if method == "least_square":
            return self.__run_least_square(data, dt_s)
        elif method == "differential_evolution":
            return self.__run_differential_evolution(data, dt_s)
        else:
            raise ValueError(f"Unsupported identification method: {method}")
    
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
        
        min_delay, max_delay = self.lower_bounds[2], self.upper_bounds[2]
        # Include both bounds and all sample-aligned delays inside them.
        delay_candidates = np.arange(np.ceil(min_delay / dt_s), np.floor(max_delay / dt_s) + 1) * dt_s
        delay_candidates = np.unique(np.clip(
            np.concatenate(([min_delay], delay_candidates, [max_delay])), min_delay, max_delay
        ))

        for delay in delay_candidates:
            
            lower_bounds    = self.lower_bounds
            upper_bounds    = self.upper_bounds
            initial_guess   = [0.25, 0.05, float(delay)]

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
