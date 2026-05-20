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
                            ):
    
    # TODO: Costumize the PID config on the left of the graph
    
    plt.figure(figsize=(10, 5))
    
    if actual_time_list is not None:
        if actual_commanded_list is not None:
            plt.plot(actual_time_list, actual_commanded_list, label=actual_commanded_label, color = "black", linestyle='--')
        
        if actual_data_list is not None:
            plt.plot(actual_time_list, actual_data_list, label=actual_data_label, color="blue", linewidth=2)
    
    if simulation_time_list is not None:
        if simulation_commanded_list is not None:
            plt.plot(simulation_time_list, simulation_commanded_list, label=simulation_commanded_label, color = "black", linestyle='--')
        if simulation_data_list is not None:
            plt.plot(simulation_time_list, simulation_data_list, label=simulation_data_label, color="red", linewidth=2)
    
    if plot_title == "":
        plt.title(f"Step Response (Kp={pid_config[0]:.2f}, Ki={pid_config[1]:.2f}, Kd={pid_config[2]:.2f} I_Limit={pid_config[3]:.0f})")
    else:
        plt.title(plot_title)
        
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    try:
        image_name = tag + "simulation_graph_" + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time())) + ".jpg"
        plt.savefig(log_dir+image_name, dpi = 300)
        
        _, sep, after = log_dir.partition("/LOG/")
        result = sep + after
        print_log("INFO", "Graph has been saved on: ", end="")
        printg(result+image_name)
        
    except Exception as e:
        printr(f"[ERROR] - {e}" )
    plt.close()