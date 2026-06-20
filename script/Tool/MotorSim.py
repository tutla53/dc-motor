import numpy as np
from bisect import bisect_left
from base_url import *
from Tool.visualize import *
from enum import Enum

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

class SimulateMotor:
    def __init__(self, configfile):
        self.config = configfile
        
        # System Parameter
        self.K_positive = self.config.K_POSITIVE       # Linear Gain
        self.K_negative = self.config.K_NEGATIVE       # Linear Gain
        self.tau        = self.config.TAU_S            # Time Constant
        self.L_STEP          = self.config.DELAY_STEPS      # Time Delay
        self.DT_S       = self.config.DT_S             # Time Sampling
        
        # Nonlinear Motor Gain
        self.K_LIST     =  self.config.K_LIST       # Nonlinear Gain Based on PWM Input
        self.TAU_LIST   =  self.config.TAU_LIST 
        self.PWM_LIST   =  self.config.PWM_LIST
        
        # System Config
        self.RPM_PER_PPS            = (60.0*self.config.ROTATION_PER_PULSE)
        self.PPS_PER_RPM            = 1 / self.RPM_PER_PPS
        self.ROTATION_PER_PULSE     = self.config.ROTATION_PER_PULSE
        self.PULSE_PER_ROTATION     = 1 / self.ROTATION_PER_PULSE
        self.MAX_PWM                = self.config.MAX_PWM_TICKS
        self.MAX_SPEED_PPS          = self.config.MAX_SPEED_PPS
        self.MAX_SPEED_RPM          = self.config.MAX_SPEED_RPM
        
        # Linear Discrete Model System
        self.ALPHA          = np.exp(-self.DT_S / self.tau)
        self.BETA_POSITIVE  = self.K_positive * (1 - self.ALPHA)
        self.BETA_NETAGIVE  = self.K_negative * (1 - self.ALPHA)
    
    def __interpolate__(self, data_list, input_pwm):    
        
        if input_pwm <= self.PWM_LIST[0]:
            return data_list[0]
        if input_pwm >= self.PWM_LIST[-1]:
            return data_list[-1]
        
        idx = bisect_left(self.PWM_LIST, input_pwm)
        
        p0, p1 = self.PWM_LIST[idx-1], self.PWM_LIST[idx]
        k0, k1 = data_list[idx-1], data_list[idx]
        weight = (input_pwm - p0) / (p1 - p0)
        K = k0 + weight * (k1 - k0)
        
        return K

    def simulate_open_loop_response(self, target_pwm, u: list,  mode: SystemModel = SystemModel.Linear):
        # Simulation Variable
        d   = self.L_STEP                        # Time Delay
        N   = len(u)                        # Number of Data
        y   = [0.0] * N                     # Output: Motor Speed (Pulse per Second)
        t   = np.linspace(0, N * self.DT_S, N)   # Time Array    
        
        target_pwm = np.clip(target_pwm, -self.MAX_PWM, self.MAX_PWM)
        
        if mode == SystemModel.Linear:            
            for k in range(N):
                if (k - d - 1) < 0:
                    continue
                
                ALPHA   = self.ALPHA
                BETA    = self.BETA_POSITIVE if u[k-d-1] > 0 else self.BETA_NETAGIVE                
                
                y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]             # Difference Equation: Update Speed
        
        elif mode == SystemModel.Nonlinear:
            for k in range(N):
                
                if (k - d - 1) < 0:
                    continue
                
                K_intrp = self.__interpolate__(self.K_LIST, u[k-d-1])
                ALPHA   = self.ALPHA
                BETA    = K_intrp * (1 - ALPHA)
                
                y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]             # Difference Equation: Update Speed
        
        return t, np.array(y)*self.RPM_PER_PPS , np.array(u)
    
    def simulate_pid_speed_control(self, pid_params, initial_speed, setpoint:list, mode: SystemModel = SystemModel.Linear):
        """
            Target unit                 --> RPM
            PID speed calculation unit  --> pulse per second
            Speed output unit           --> RPM
        """
        
        # Simulation Variable
        d   = self.L_STEP                            # Time Delay
        N   = len(setpoint)                     # Number of Data
        t   = np.linspace(0, N*self.DT_S, N)    # Time Array    
        y   = [initial_speed] * N               # Output: Motor Speed (Pulse per Second)
        u   = [0.0] * N                         # Input: PWM (Ticks)
        
        # PID Speed Handler
        kp, ki, kd, i_limit = pid_params        
        speed_control = PIDControl(kp, ki, kd, i_limit, self.MAX_PWM)
        
        if mode == SystemModel.Linear:            
            for k in range(N):
                if (k - d - 1) < 0:
                        continue
                
                ALPHA   = self.ALPHA
                BETA    = self.BETA_POSITIVE if u[k-d-1] > 0 else self.BETA_NETAGIVE
                                
                error = setpoint[k] - y[k-1]
                u[k] = speed_control.compute(error)         # Compute PID Control
                
                y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]     # Difference Equation: Update Speed
        
        elif mode == SystemModel.Nonlinear:
            for k in range(N):
                
                if (k - d - 1) < 0:
                    continue
                
                K_intrp = self.__interpolate__(self.K_LIST, u[k-d-1])
                ALPHA   = self.ALPHA
                BETA    = K_intrp * (1 - ALPHA)
                
                error = setpoint[k] - y[k-1]
                u[k] = speed_control.compute(error)         # Compute PID Control
                
                y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]     # Difference Equation: Update Speed
                            
        return t, np.array(y)*self.RPM_PER_PPS, np.array(setpoint)*self.RPM_PER_PPS

    def simulate_pid_pos_control(self, pid_pos_params, pid_speed_params, initial_pos, setpoint:list, mode: SystemModel = SystemModel.Linear):
        """
            Target unit                 --> rotation
            PID pos calculation unit    --> pulse
            PID speed calculation unit  --> pulse per seconds
            Speed output unit           --> RPM
            Position output unit        --> rotation
        """

        # Simulation Variable
        d   = self.L_STEP                        # Time Delay
        N   = len(setpoint)                        # Number of Data
        t   = np.linspace(0, N*self.DT_S, N)   # Time Array    
        y   = [0.0] * N                     # Output: Motor Speed (Pulse per Second)
        u   = [0.0] * N                     # Input: PWM (Ticks)
        x   = [initial_pos] * N                     # Motor Position List (Pulse)        
        
        # PID Handler
        kp_speed, ki_speed, kd_speed, i_limit_speed = pid_speed_params
        kp_pos, ki_pos, kd_pos, i_limit_pos = pid_pos_params

        speed_control       = PIDControl(kp_speed, ki_speed, kd_speed, i_limit_speed, self.MAX_PWM)
        position_control    = PIDControl(kp_pos, ki_pos, kd_pos, i_limit_pos, self.MAX_SPEED_PPS)
        
        if mode == SystemModel.Linear:
            
            for k in range(N):
                if (k - d - 1) < 0:
                        continue
                
                ALPHA   = self.ALPHA
                BETA    = self.BETA_POSITIVE if u[k-d-1] > 0 else self.BETA_NETAGIVE
                
                pos_error = setpoint[k] - x[k-1]
                target_speed = position_control.compute(pos_error)  # Compute PID Position Control
                
                speed_error = target_speed - y[k-1]
                u[k] = speed_control.compute(speed_error)           # Compute PID Speed Control  
                
                y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]             # Difference Equation: Update Speed
                
                x[k] = x[k-1] + ((y[k-1] + y[k])/2.0) * self.DT_S   # Update Position

        elif mode == SystemModel.Nonlinear:
            for k in range(N):
                
                if (k - d - 1) < 0:
                    continue
                
                K_intrp = self.__interpolate__(self.K_LIST, u[k-d-1])
                ALPHA   = self.ALPHA
                BETA    = K_intrp * (1 - ALPHA)
                
                pos_error = setpoint[k] - x[k-1]
                target_speed = position_control.compute(pos_error)  # Compute PID Position Control
                
                speed_error = target_speed - y[k-1]
                u[k] = speed_control.compute(speed_error)           # Compute PID Speed Control  
                
                y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]             # Difference Equation: Update Speed
                
                x[k] = x[k-1] + ((y[k-1] + y[k])/2.0) * self.DT_S   # Update Position
                            
        return t, np.array(x)*self.ROTATION_PER_PULSE, np.array(setpoint)*self.ROTATION_PER_PULSE, np.array(y)* self.RPM_PER_PPS
    
