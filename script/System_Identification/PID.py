import numpy as np
import matplotlib.pyplot as plt

# --- System Parameters ---
K = 0.24126270336163416
tau = 0.04858428774840934
dt = 0.005  # 200 Hz (TIME_SAMPLING_US = 5000)

# --- Motor Physics (Discrete) ---
alpha = np.exp(-dt / tau)
beta = K * (1 - alpha)

# --- PID Configuration (Recommended Balanced) ---
kp = 10.0
ki = 0.85  # Scaled for 200Hz
kd = 2.0    # Start with 0 to stabilize
i_limit = 5319.0  # Reduced from 5319 to prevent overshoot
max_pwm = 5319.0

def simulate():
    duration = 1.5
    steps = int(duration / dt)
    t = np.linspace(0, duration, steps)
    
    current_speed = 0.0
    integral = 0.0
    prev_error = 0.0
    
    speed_history = []
    setpoint_history = []
    
    for i in range(steps):
        setpoint = 600.0 if t[i] >= 0.4 else 0.0
        error = setpoint - current_speed
        
        # --- RUST IMPLEMENTATION LOGIC ---
        # next_integral = self.integral.saturating_add(error);
        # self.integral = next_integral.clamp(-self.i_limit, self.i_limit);
        integral += error
        integral = np.clip(integral, -i_limit, i_limit)
        
        # let derivative = error - self.prev_error;
        derivative = error - prev_error
        prev_error = error
        
        # let sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);
        pwm_out = (kp * error) + (ki * integral) + (kd * derivative)
        pwm_out = np.clip(pwm_out, -max_pwm, max_pwm)
        
        # --- Physical Response ---
        current_speed = (alpha * current_speed) + (beta * pwm_out)
        
        speed_history.append(current_speed)
        setpoint_history.append(setpoint)
        
    return t, speed_history, setpoint_history

t, speed, target = simulate()

plt.figure(figsize=(10, 5))
plt.plot(t, speed, label="Simulated Speed (RPM)", color='blue', linewidth=2)
plt.step(t, target, label="Target", color='red', linestyle='--')
plt.title(f"200Hz PI Simulation (Kp={kp}, Ki={ki}, Kd={kd} I_Limit={i_limit})")
plt.ylabel("RPM")
plt.xlabel("Time (s)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()