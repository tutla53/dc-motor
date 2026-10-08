from pathlib import Path
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

from Tool.visualize import printg, printy, print_log
from base_url import base_url


def find_regions(pwm, gain, speed, direction):
    """Find regions on one side, walking from low to high PWM magnitude.

    Missing regions are None. Retain the existing 15 PPS deadband and
    relative-change thresholds (2.5% gain, 2% speed).
    """
    indices = np.flatnonzero(pwm * direction > 0)
    indices = indices[np.argsort(np.abs(pwm[indices]), kind="stable")]
    dead = np.flatnonzero(np.abs(speed[indices]) < 15)
    dead_offset = int(dead[-1]) if dead.size else None
    dead_index = int(indices[dead_offset]) if dead_offset is not None else None
    linear_start = linear_stop = None
    start = dead_offset + 2 if dead_offset is not None else 1
    for offset in range(start, len(indices)):
        previous, current = indices[offset - 1], indices[offset]
        if gain[current] == 0:
            break
        delta = abs((gain[previous] - gain[current]) / gain[current])
        if delta < 0.025:
            if linear_start is None:
                linear_start = offset
            linear_stop = offset
        elif linear_start is not None:
            break

    linear = None
    if linear_start is not None:
        candidate = indices[linear_start:linear_stop + 1]
        if len(candidate) >= 2 and np.unique(pwm[candidate]).size >= 2:
            linear = candidate

    saturation = None
    if linear is not None:
        for offset in range(linear_stop + 2, len(indices)):
            previous, current = indices[offset - 1], indices[offset]
            if speed[current] == 0:
                continue
            delta = abs((speed[previous] - speed[current]) / speed[current])
            if delta < 0.020:
                saturation = int(current)
                break
    return dead_index, linear, saturation


def create_system_identification_graph(filename, config, log_dir="", tag=""):
    data = pd.read_csv(filename)
    required = ["PWM", "K", "tau", "L"]
    missing = set(required).difference(data.columns)
    if missing:
        raise ValueError(f"Missing result columns: {sorted(missing)}")
    if data.empty:
        print_log("WARN", "No identification results to plot.")
        return
    values = data[required].to_numpy(dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("Identification results contain non-finite values")
    if not np.isfinite(config.MAX_PWM_TICKS) or config.MAX_PWM_TICKS <= 0:
        raise ValueError("MAX_PWM_TICKS must be finite and positive")
    if not np.isfinite(config.ROTATION_PER_PULSE) or config.ROTATION_PER_PULSE <= 0:
        raise ValueError("ROTATION_PER_PULSE must be finite and positive")

    values = values[np.argsort(values[:, 0], kind="stable")]
    pwm, gain, tau, delay = values.T
    speed = pwm * gain
    rpm_scale = config.ROTATION_PER_PULSE * 60
    pwm_percent = pwm / config.MAX_PWM_TICKS * 100
    output_dir = Path(log_dir) if log_dir else Path(base_url) / "LOG"
    output_dir.mkdir(parents=True, exist_ok=True)
    tag = tag or time.strftime('%Y%m%d%H%M%S', time.localtime())

    fig, ax = plt.subplots(figsize=(10, 6))
    try:
        ax.plot(pwm_percent, speed * rpm_scale, label="Motor Response", color="blue", marker=".")
        printy("System Identification Result")
        labeled_zones = set()
        for direction, label in ((1, "Positive"), (-1, "Negative")):
            dead, linear, saturation = find_regions(pwm, gain, speed, direction)
            zones = []
            if dead is not None:
                zones.append((0, pwm_percent[dead], "red", "Zone 1: Deadband"))
            if linear is None:
                print_log("WARN", f"{label}: insufficient linear-region data; fit and speed limits omitted.")
            else:
                fit = stats.linregress(pwm[linear], speed[linear])
                mean_gain = np.mean(gain[linear])
                ax.plot(pwm_percent[linear], (pwm[linear] * fit.slope + fit.intercept) * rpm_scale,
                        color="red", label=f"K_{label.lower()}: {mean_gain:.4f}")
                print_log("INFO", f"K_{label}: {mean_gain:.4f}")
                print_log("INFO", f"[{label}] tau: {np.mean(tau[linear]):.4f} s, Time Delay: {np.mean(delay[linear]):.3f} s")
                first, last = linear[0], linear[-1]
                print_log("INFO", f"[{label}] Min Speed: {speed[first]:.0f} PPS, Max Speed: {speed[last]:.0f} PPS")
                if dead is not None:
                    zones.append((pwm_percent[dead], pwm_percent[first], "orange", "Zone 2: Nonlinear Transition"))
                zones.append((pwm_percent[first], pwm_percent[last], "green", "Zone 3: Linear"))
                if saturation is not None:
                    zones.append((pwm_percent[last], pwm_percent[saturation], "yellow", "Zone 4: Pre-saturation"))
                    zones.append((pwm_percent[saturation], direction * 100, "purple", "Zone 5: Saturation"))
            for left, right, color, name in zones:
                ax.axvspan(min(left, right), max(left, right), color=color, alpha=0.1,
                           label=name if name not in labeled_zones else None)
                labeled_zones.add(name)

        ax.axhline(0, color="black")
        ax.axvline(0, color="black")
        ax.set(xlabel="PWM Input (%)", ylabel="Motor Speed (RPM)", title="Motor Linearity",
               ylim=(-1500, 1500), xlim=(-100, 100))
        ax.legend()
        ax.grid()
        image_path = output_dir / f"Motor_Linearity_{tag}.jpg"
        fig.savefig(image_path, dpi=300)
        print_log("INFO", "Graph has been saved on:", end=" ")
        printg(image_path)
    finally:
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 6))
    try:
        ax.plot(pwm_percent, gain, label="Steady-State Gain", marker=".")
        ax.plot(pwm_percent, tau, label="Time-Constant (s)", marker=".")
        ax.plot(pwm_percent, delay, label="Time-Delay (s)", marker=".")
        ax.axhline(0, color="black")
        ax.axvline(0, color="black")
        ax.set(xlabel="PWM Input (%)", ylabel="Parameter Value", title="System Identification Result",
               xlim=(-100, 100))
        ax.legend()
        ax.grid()
        image_path = output_dir / f"System_Identification_{tag}.jpg"
        fig.savefig(image_path, dpi=300)
        print_log("INFO", "Graph has been saved on:", end=" ")
        printg(image_path)
    finally:
        plt.close(fig)
