import matplotlib.pyplot as plt
import pandas as pd
import time
import numpy as np

from scipy import stats
from Tool.visualize import *
from base_url import *

def save_image_by_tag(tag):
    if tag == "":
        print_log("ERROR", "Invalid File Tag")
        return False
    else:
        log_dir = base_url+"/LOG/"+tag+"/"
        all_log_files = os.listdir(log_dir)

        for file in all_log_files:
            save_image(log_dir, file)

def save_image(log_dir, filename):
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
            
    data = pd.read_csv(log_dir+filename)
    t = data["Timestamp(ms)"]/1000.0
    column_names = []
    tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
    plot_mode = "Position"

    for column in data.columns:
        if column == "Timestamp(ms)":
            continue
        if "Speed" in column:
            plot_mode = "Speed"
        column_names.append(column)

    for column in column_names:
        plt.plot(t, data[column], label = column)
    
    if plot_mode == "Position":
        plt.ylabel("Position (Rotation)")
        plt.title("Position Response")

    elif plot_mode == "Speed":
        plt.ylabel("Speed (RPM)")
        plt.title("Speed Response")

    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid()
    
    try:
        image_name = "graph_"+tag+".jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        printr(f"[ERROR] - {e}" )
    plt.close()

def create_simulation_plot( log_dir, pid_config, 
                            simulation_time_list: list|None, 
                            actual_time_list: list|None, 
                            simulation_data_list: list|None, 
                            simulation_commanded_list: list|None, 
                            actual_data_list: list|None, 
                            actual_commanded_list: list|None, 
                            tag="",
                            plot_title ="",
                            y_label = "RPM",
                            x_label = "Time (s)",
                            actual_data_label = "Actual Speed",
                            actual_commanded_label = "Actual Command",
                            simulation_data_label = "Simulation Speed",
                            simulation_commanded_label = "Simulation Command",
                            same_unit = True,
                            ):
    
    # TODO: Costumize the PID config on the left of the graph
    
    fig, ax1 = plt.subplots(figsize=(10, 5))
    lines = []
    found_commanded = False
    commanded_data = None
    
    commanded_axis = ax1
    if not same_unit:
        commanded_axis = ax1.twinx()
        commanded_axis.set_ylabel("PWM Input (Ticks)")
    
    if actual_time_list is not None:
        if actual_commanded_list is not None:
            line = commanded_axis.plot(actual_time_list, actual_commanded_list, label=actual_commanded_label, color = "black", linestyle='--')
            lines += line
            commanded_data = actual_commanded_list.tolist()
            found_commanded = True
        
        if actual_data_list is not None:
            line = ax1.plot(actual_time_list, actual_data_list, label=actual_data_label, color="blue", linewidth=2)
            lines += line
    
    if simulation_time_list is not None:
        if simulation_commanded_list is not None and not found_commanded:
            line = commanded_axis.plot(simulation_time_list, simulation_commanded_list, label=simulation_commanded_label, color = "black", linestyle='--')
            commanded_data = simulation_commanded_list.tolist()
            lines += line
            
        if simulation_data_list is not None:
            line = ax1.plot(simulation_time_list, simulation_data_list, label=simulation_data_label, color="red", linewidth=2)
            lines += line
    
    if pid_config is not None:
        plot_title += f"\n(Kp={pid_config[0]:.2f}, Ki={pid_config[1]:.2f}, Kd={pid_config[2]:.2f} I_Limit={pid_config[3]:.0f})"

    plt.title(plot_title)
    
    labels = [l.get_label() for l in lines]
    
    ax1.legend(lines, labels)
    ax1.set_ylabel(y_label)
    
    if not same_unit and simulation_data_list is not None and commanded_data is not None:
        if max(abs(simulation_data_list)) != 0:
            going_positive = True if (commanded_data[-10]-commanded_data[0]) > 0 else False

            low, up = ax1.get_ybound()
            low_2, up_2 = commanded_axis.get_ybound()           
            
            if going_positive:
                new_scale = ((up - low) * (max(commanded_data)/max(simulation_data_list))) + low_2
                commanded_axis.set_ybound(low_2, new_scale)
            else:
                new_scale = -(((up - low) * (min(commanded_data)/min(simulation_data_list))) - up_2)
                commanded_axis.set_ybound(new_scale, up_2)            
            
    ax1.set_xlabel(x_label)
    plt.grid(True, alpha=0.3)
    
    try:
        image_name = tag + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time())) + ".jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        printr(f"[ERROR] - {e}" )
    plt.close()
    
def create_system_identification_graph(filename, config, log_dir="", tag=""):
    data        = pd.read_csv(filename)
    PWM_LIST    = data["PWM"].values
    K_LIST      = data["K"].values
    TAU_LIST    = data["tau"].values
    L_LIST      = data["L"].values
    PPS_LIST    = PWM_LIST * K_LIST
    RPM_LIST    = PWM_LIST * K_LIST * config.ROTATION_PER_PULSE * 60
    
    DEADBAND_LIMIT          = 15    # PPS
    DELTA_K_LIMIT           = 0.025 # Percent
    DELTA_SPEED_LIMIT       = 0.020 # Percent
    
    dead_zone_pos_idx       = 0
    dead_zone_neg_idx       = 0
    linear_pos_start        = 0
    linear_pos_stop         = 0
    linear_neg_start        = 0
    linear_neg_stop         = 0
    saturation_pos_start    = 0
    saturation_neg_start    = 0
    
    # COVER BOTH POSITIVE AND NEGATIVE
    for idx, speed in enumerate(PPS_LIST):
        if speed < 0 and dead_zone_neg_idx==0:
            if speed > -DEADBAND_LIMIT:
                dead_zone_neg_idx = idx
        elif speed > 0:
            if speed < DEADBAND_LIMIT:
                dead_zone_pos_idx = idx
    
    # Linear Region - Positive Range
    for idx in range(dead_zone_pos_idx+2, len(K_LIST)):
        if idx-1 < 0:
            continue
        
        delta_K = abs((K_LIST[idx-1] - K_LIST[idx]) / K_LIST[idx])
        
        if delta_K < DELTA_K_LIMIT:
            if linear_pos_start == 0:
                linear_pos_start = idx
            linear_pos_stop = idx

        if linear_pos_stop != 0 and delta_K > DELTA_K_LIMIT:
            break
    
    # Linear Region - Positive Range
    idx = dead_zone_neg_idx - 2
    while idx > 0:
        
        delta_K = abs((K_LIST[idx-1] - K_LIST[idx]) / K_LIST[idx])
        
        if delta_K < DELTA_K_LIMIT:
            if linear_neg_start == 0:
                linear_neg_start = idx
            linear_neg_stop = idx

        if linear_neg_stop != 0 and delta_K > DELTA_K_LIMIT:
            break
        idx -= 1
    
    # Saturation Region - Positive Range
    for idx in range(linear_pos_stop+2, len(PPS_LIST)):
        if idx-1 < 0:
            continue
        
        delta_speed = abs((PPS_LIST[idx-1] - PPS_LIST[idx]) / PPS_LIST[idx])
        
        if delta_speed < DELTA_SPEED_LIMIT:
            saturation_pos_start = idx
            break
    
    # Saturation Region - Negative Range
    idx = linear_neg_stop-2
    while idx > 0:
        delta_speed = abs((PPS_LIST[idx-1] - PPS_LIST[idx]) / PPS_LIST[idx])
        
        if delta_speed < DELTA_SPEED_LIMIT:
            saturation_neg_start = idx
            break
        idx -=1
    
    x_region = PWM_LIST[linear_pos_start:linear_pos_stop+1]
    y_region = PPS_LIST[linear_pos_start:linear_pos_stop+1]
    slope_positive, c_pos, _, _, _ = stats.linregress(x_region, y_region, "greater")
    K_POS = np.mean(K_LIST[linear_pos_start:linear_pos_stop+1])
    
    x_region = PWM_LIST[linear_neg_stop:linear_neg_start+1]
    y_region = PPS_LIST[linear_neg_stop:linear_neg_start+1]
    slope_negative, c_neg, _, _, _ = stats.linregress(x_region, y_region, "greater")
    K_NEG = np.mean(K_LIST[linear_neg_stop:linear_neg_start+1])
    
    ###### PRINT RESUME ######
    printy("System Identification Result")
    print_log("INFO", f"K_Positive: {K_POS:.4f}")
    print_log("INFO", f"K_Negative: {K_NEG:.4f}")
    print_log("INFO", f"tau:        {np.mean(TAU_LIST[linear_pos_start:linear_pos_stop+1]):.4f}")
    print_log("INFO", f"Time Delay: {np.mean(L_LIST[linear_pos_start:linear_pos_stop+1]):.3f}")
    
    printy("Motor Limit")
    print_log("INFO", f"[Positive] Min Speed: {PPS_LIST[linear_pos_start]:.0f} PPS, Max Speed: {PPS_LIST[linear_pos_stop]:.0f} PPS")
    print_log("INFO", f"[Negative] Min Speed: {PPS_LIST[linear_neg_start]:.0f} PPS, Max Speed: {PPS_LIST[linear_neg_stop]:.0f} PPS")
    
    # Plot
    zone_1_pos = (PWM_LIST[dead_zone_pos_idx]/config.MAX_PWM_TICKS)*100
    zone_2_pos = (PWM_LIST[linear_pos_start]/config.MAX_PWM_TICKS)*100
    zone_3_pos = (PWM_LIST[linear_pos_stop]/config.MAX_PWM_TICKS)*100
    zone_4_pos = (PWM_LIST[saturation_pos_start]/config.MAX_PWM_TICKS)*100
    zone_5_pos = 100
    
    zone_1_neg = (PWM_LIST[dead_zone_neg_idx]/config.MAX_PWM_TICKS)*100
    zone_2_neg = (PWM_LIST[linear_neg_start]/config.MAX_PWM_TICKS)*100
    zone_3_neg = (PWM_LIST[linear_neg_stop]/config.MAX_PWM_TICKS)*100
    zone_4_neg = (PWM_LIST[saturation_neg_start]/config.MAX_PWM_TICKS)*100
    zone_5_neg = -100
    
    plt.figure(figsize=(10, 6))
    plt.plot((PWM_LIST/config.MAX_PWM_TICKS)*100, RPM_LIST, label='Motor Response', color="blue", marker=".")
    plt.plot((PWM_LIST[linear_pos_start: linear_pos_stop+1]/config.MAX_PWM_TICKS)*100, (PWM_LIST[linear_pos_start: linear_pos_stop+1] * slope_positive + c_pos) * config.ROTATION_PER_PULSE * 60, color="red", label=f"K_pos: {K_POS:.4f}")
    plt.plot((PWM_LIST[linear_neg_stop: linear_neg_start+1]/config.MAX_PWM_TICKS)*100, (PWM_LIST[linear_neg_stop: linear_neg_start+1] * slope_negative + c_neg) * config.ROTATION_PER_PULSE * 60, color="red", label=f"K_neg: {K_NEG:.4f}")
    
    plt.axhline(0, color="black")
    plt.axvline(0, color="black")
    
    plt.axvspan(0, zone_1_pos, color='red', alpha=0.1, label='Zone 1: Deadband')
    plt.axvspan(zone_1_pos, zone_2_pos, color='orange', alpha=0.1, label='Zone 2: Nonlinear Transition')
    plt.axvspan(zone_2_pos, zone_3_pos, color='green', alpha=0.1, label='Zone 3: Linear')
    plt.axvspan(zone_3_pos, zone_4_pos, color='yellow', alpha=0.1, label='Zone 4: Pre-saturation')
    plt.axvspan(zone_4_pos, zone_5_pos, color='purple', alpha=0.1, label='Zone 5: Saturation')
    
    plt.axvspan(zone_1_neg, 0, color='red', alpha=0.1)
    plt.axvspan(zone_2_neg, zone_1_neg, color='orange', alpha=0.1)
    plt.axvspan(zone_3_neg, zone_2_neg, color='green', alpha=0.1)
    plt.axvspan(zone_4_neg, zone_3_neg, color='yellow', alpha=0.1)
    plt.axvspan(zone_5_neg, zone_4_neg, color='purple', alpha=0.1)
    
    
    plt.xlabel('PWM Input (%)', fontsize=12)
    plt.ylabel('Motor Speed (RPM)', fontsize=12)
    plt.title(f"Motor Linearity", fontsize=14)
    plt.ylim([-1500, 1500])
    plt.xlim([-100, 100])
    plt.legend()
    plt.grid()

    try:
        if log_dir == "":
            log_dir = base_url+"/LOG/"
        if tag == "":
            tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        image_name = "Motor_Linearity_" + tag +".jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        print(f"[ERROR] - {e}" )
    plt.close()
    
    plt.figure(figsize=(10, 6))
    plt.plot((PWM_LIST/config.MAX_PWM_TICKS)*100, K_LIST, label='Steady-State Gain', marker=".")
    plt.plot((PWM_LIST/config.MAX_PWM_TICKS)*100, TAU_LIST, label='Time-Constant (s)', marker=".")
    plt.plot((PWM_LIST/config.MAX_PWM_TICKS)*100, L_LIST, label='Time-Delay (s)', marker=".")
    
    plt.axhline(0, color="black")
    plt.axvline(0, color="black")
    
    plt.xlabel('PWM Input (%)', fontsize=12)
    plt.ylabel('Parameter Value', fontsize=12)
    plt.title(f"System Identification Result", fontsize=14)
    plt.xlim([-100, 100])
    plt.legend()
    plt.grid()
    
    try:
        if log_dir == "":
            log_dir = base_url+"/LOG/"
        if tag == "":
            tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        image_name = "System_Identification_" + tag +".jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        print(f"[ERROR] - {e}" )
    plt.close()
