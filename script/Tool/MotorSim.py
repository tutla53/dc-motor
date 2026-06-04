import numpy as np
from bisect import bisect_left
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

class SimulateMotor:
    def __init__(self, configfile):
        self.config = configfile
        
        # System Parameter
        self.K       = self.config.K                # Linear Gain
        self.tau     = self.config.TAU_S            # Time Constant
        self.L       = self.config.DELAY_STEPS      # Time Delay
        self.DT_S    = self.config.DT_S             # Time Sampling
        
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
        self.ALPHA      = np.exp(-self.DT_S / self.tau)
        self.BETA       = self.K * (1 - self.ALPHA)
    
    def __calculate_nonlinear(self, data_list, input_pwm):    
        
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
            ALPHA = self.ALPHA
            BETA = self.BETA
        elif mode == SystemModel.Nonlinear:
            K_intrp     = self.__calculate_nonlinear(self.K_LIST, input_pwm)
            tau_intrp   = self.__calculate_nonlinear(self.TAU_LIST, input_pwm)
            ALPHA       = np.exp(-self.DT_S / tau_intrp)
            BETA        = K_intrp * (1 - ALPHA)
        
        new_speed_pps = (ALPHA * current_speed_pps) + (BETA * input_pwm)
        
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
        position_control    = PIDControl(kp_pos, ki_pos, kd_pos, i_limit_pos, self.MAX_SPEED_PPS)
        
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
            current_speed_pps = self.__update_motor_speed(current_speed_pps, delayed_pwm, SystemModel.Linear)
            
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
    