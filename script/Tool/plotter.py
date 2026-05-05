import matplotlib.pyplot as plt
import pandas as pd
import time
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

def create_simulation_plot(log_dir, pid_config, simulation_time_list: list|None, actual_time_list: list|None, simulation_speed: list|None, simulation_target: list|None, actual_speed: list|None, actual_commanded: list|None):
    plt.figure(figsize=(10, 5))
    
    if (actual_time_list is None) and (simulation_time_list is None):
        printr("Invalid Time List")
        return
    
    if actual_speed is not None and actual_commanded is not None:
        if simulation_speed is not None and simulation_target is not None:
            if (actual_time_list is None) or (simulation_time_list is None):
                printr("Invalid Time List")
                return
        
            # Complete Graph
            plt.plot(actual_time_list, actual_commanded, label="Commanded Speed", color = "black", linestyle='--')
            plt.plot(actual_time_list, actual_speed, label="Actual Speed", color="blue", linewidth=2)
            plt.plot(simulation_time_list, simulation_speed, label="Simulated Speed", color="red", linewidth=2)
            
        else:
            if (actual_time_list is None):
                printr("Invalid Time List")
                return
            
            # Firmware Log Only
            plt.plot(actual_time_list, actual_commanded, label="Commanded Speed", color = "black", linestyle='--')
            plt.plot(actual_time_list, actual_speed, label="Actual Speed", color="blue", linewidth=2)
    
    elif (simulation_speed is not None) and (simulation_target is not None):
        if (simulation_time_list is None):
            printr("Invalid Time List")
            return
        
        # Simulation Only
        plt.plot(simulation_time_list, simulation_target, label="Commanded Speed", color = "black", linestyle='--')
        plt.plot(simulation_time_list, simulation_speed, label="Simulated Speed", color="red", linewidth=2)
    
    else:
        printr("Invalid parameter")
        return
    
    plt.title(f"Step Speed Response (Kp={pid_config["kp"]:.2f}, Ki={pid_config["ki"]:.2f}, Kd={pid_config["kd"]:.2f} I_Limit={pid_config["i_limit"]:.0f})")
    plt.ylabel("RPM")
    plt.xlabel("Time (s)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    try:
        
        image_name = "simulation_graph.jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        printr(f"[ERROR] - {e}" )
    plt.close()