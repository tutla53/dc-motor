import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. System Identification & Hardware Config
# ==========================================
K = 0.24126270336163416
TAU = 0.04858428774840934
DT = 0.005  # 200 Hz (TIME_SAMPLING_US = 5000)

# Measured from image_fea915.jpg: ~40ms delay
DEAD_TIME_S = 0.02 
DELAY_STEPS = int(DEAD_TIME_S / DT)

# Motor Physics (Discrete-time approximation)
# y[k] = alpha * y[k-1] + beta * u[k-delay]
ALPHA = np.exp(-DT / TAU)
BETA = K * (1 - ALPHA)

# ==========================================
# 2. Rust-Equivalent PID Controller
# ==========================================
class RustPID:
    def __init__(self, kp, ki, kd, i_limit, max_pwm):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.max_threshold = max_pwm
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        # next_integral = self.integral.saturating_add(error)
        # self.integral = next_integral.clamp(-self.i_limit, self.i_limit)
        self.integral = np.clip(self.integral + error, -self.i_limit, self.i_limit)

        # let derivative = error - self.prev_error
        derivative = error - self.prev_error
        self.prev_error = error

        # let sig = (kp * error) + (ki * integral) + (kd * derivative)
        sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative_logic(derivative))
        
        # .clamp(-self.max_threshold, self.max_threshold)
        return np.clip(sig, -self.max_threshold, self.max_threshold)

    def derivative_logic(self, d):
        return d # Pure raw derivative as per your Rust snippet

# ==========================================
# 3. Simulation Loop
# ==========================================
def run_simulation(kp, ki, kd, i_limit):
    duration = 1.5 # seconds
    steps = int(duration / DT)
    t = np.linspace(0, duration, steps)
    
    pid = RustPID(kp, ki, kd, i_limit, 5319.0)
    
    speed_history = []
    pwm_buffer = [0.0] * DELAY_STEPS # Hardware Latency Buffer
    current_speed = 0.0

    for i in range(steps):
        # Step input at 0.4s
        setpoint = 600.0 if t[i] >= 0.4 else 0.0
        error = setpoint - current_speed
        
        # PID calculates output based on CURRENT error
        pwm_out = pid.compute(error)
        
        # Apply Latency: Motor only sees what happened X steps ago
        pwm_buffer.append(pwm_out)
        delayed_pwm = pwm_buffer.pop(0)
        
        # Physics Update
        current_speed = (ALPHA * current_speed) + (BETA * delayed_pwm)
        speed_history.append(current_speed)
        
    return t, speed_history

# ==========================================
# 4. Compare Current vs. Suggested
# ==========================================
# CURRENT (Your unstable settings from image_fea915.jpg)
t, speed_unstable = run_simulation(kp=1.0, ki=1.0, kd=0.0, i_limit=5319.0)

# SUGGESTED (Corrected for Latency/Delay)
t, speed_stable = run_simulation(kp=10.0, ki=0.0, kd=0.0, i_limit=5319.0)

# Plotting
plt.figure(figsize=(12, 6))
# plt.plot(t, speed_unstable, label="Current Settings (Unstable)", color='red', alpha=0.6)
plt.plot(t, speed_stable, label="Suggested Settings (Balanced)", color='green', linewidth=2)
plt.axhline(600, color='black', linestyle='--', label="Setpoint")
plt.title(f"Simulation with {int(DEAD_TIME_S*1000)}ms Latency (Reality Gap Correction)")
plt.xlabel("Time (s)")
plt.ylabel("Speed (RPM)")
plt.legend()
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.show()