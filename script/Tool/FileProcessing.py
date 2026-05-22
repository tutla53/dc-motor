import numpy as np
import pandas as pd

def extract_firmware_log_data(log_path, command_header, data_header, threshold_percent=2.0):
    data = pd.read_csv(log_path)
    dt_ms = data["Timestamp(ms)"][1] - data["Timestamp(ms)"][0]
    dt_s = dt_ms/1000.0
    t = data["Timestamp(ms)"]/1000.0
    
    data_list       = data[data_header]
    command_list    = data[command_header]
    
    target = np.max(np.abs(command_list))
    
    if command_list[0] > command_list[len(command_list)-1] :
        target = -target
        
    duration = len(t)
    
    start_time, settling_time = calculate_settling_time(t, data_list, command_list, 2)
    
    output = {  "target"        : target,
                "start_time_s"  : start_time,
                "duration_s"    : (duration*dt_s),
                "dt_s"          : dt_s,
                "command_list"  : command_list,
                "data_list"     : data_list,
                "time_list"     : t,
                "t_set"         : settling_time,        
            }
    
    return output

def calculate_settling_time(time_list, data_list, command_list, threshold_percent=2.0):

    t    = np.asarray(time_list)
    data    = np.asarray(data_list)
    command = np.asarray(command_list)
    
    step_indices = np.where(command != command[0])[0]
    if len(step_indices) == 0:
        return None, None
    
    start_idx = step_indices[0]
    start_time = t[start_idx]
    final_setpoint = command[-1]
    allowed_error = abs(final_setpoint * (threshold_percent / 100.0))
    
    absolute_error = np.abs(data - final_setpoint)

    outside_band_indices = np.where(absolute_error[start_idx:] > allowed_error)[0]
    
    if len(outside_band_indices) == 0:
        return start_time, 0.0
        
    last_outside_idx = outside_band_indices[-1] + start_idx
    if last_outside_idx >= len(t) - 1:
        return start_time, None

    settled_time_stamp = t[last_outside_idx + 1]
    settling_time = settled_time_stamp - start_time
    
    return start_time, settling_time