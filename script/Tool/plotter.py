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