import numpy as np
import pandas as pd

def extract_firmware_log_data(log_path, commanded_header, log_header):
    data = pd.read_csv(log_path)
    dt_ms = data["Timestamp(ms)"][1] - data["Timestamp(ms)"][0]
    dt_s = dt_ms/1000.0
    t = data["Timestamp(ms)"]/1000.0
    actual_speed = data[log_header]
    commanded = data[commanded_header]
    
    target = np.max(np.abs(commanded))
    duration = len(t)
    
    start_time = 0
    initial_pos = commanded[0]
    RPM_TRESHOLD = 10
    for i in range(1, len(t)):
        if np.abs(commanded[i]-initial_pos) > RPM_TRESHOLD:
            start_time = i
            break
            
    return (target, (start_time*dt_s), (duration*dt_s), dt_s, commanded, actual_speed, t)