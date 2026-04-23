import matplotlib.pyplot as plt
import pandas as pd
import time
from Tool.visualize import *
from base_url import *

def save_image_by_tag(tag):
    log_dir = base_url+"/LOG/"
    all_log_files = os.listdir(log_dir)
    
    for file in all_log_files:
        if tag in file:
            save_image(log_dir+file)

def save_image(path="LOG/20250409103638.csv"):
    if not os.path.exists(base_url+"/IMG/"):
        os.makedirs(base_url+"/IMG/")
            
    data = pd.read_csv(path)
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
        plt.savefig(base_url+"/IMG/graph_"+tag+".jpg", dpi = 300)
        printg("[SAVED] - /IMG/graph_"+tag+".jpg")
    except Exception as e:
        printr(f"[ERROR] - {e}" )
    plt.close()