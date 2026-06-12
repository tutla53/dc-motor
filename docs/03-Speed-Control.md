# PID Speed Control

<div align="center">
  <a href="02-System-Identification.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="04-Position-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  System Identification
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="740" height="1">
  Position Control
</div>
    
#

## Algorithm

### Discrete-Time PID Control Model
The implementation of the PID on simulation is exactly the same with the PID implementation on the firmware. The code below shows the PID implementation on the Python. 

```python
import numpy as np

class PIDControl():
    def __init__(self, kp, ki, kd, i_limit, max_threshold):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.i_limit        = i_limit
        self.integral       = 0
        self.prev_error     = 0
        self.max_threshold  = max_threshold
        
    def compute(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
        
        derivative = error - self.prev_error
        self.prev_error = error
        
        pwm_out = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        pwm_out = np.clip(pwm_out, -self.max_threshold, self.max_threshold)
        return pwm_out
```
### Integrating with Discrete Motor Model
The code below shows how to integrate the PID calculation with our discrete motor model. 

```python
import numpy as np

def simulate_pid_speed_control(motor_params, pid_params, target_speed, start_time, duration):
    """
        Target unit                 --> pulse per second
        PID speed calculation unit  --> pulse per second
        Speed output unit           --> pulse per second
    """
    
    K, tau, L = motor_params

    # Simulation Variable
    d   = int(L / dt)                   # Time Delay
    N   = int(duration / dt)            # Number of Data
    t   = np.linspace(0, duration, N)   # Time Array    
    y   = [0.0] * N                     # Output: Motor Speed (Pulse per Second)
    u   = [0.0] * N                     # Input: PWM (Ticks)
    setpoint = [0.0] * N                # Setpoint List (Pulse per Second)
    
    # Limiting Speed Input Based on the Motor Top Speed
    target_speed = np.clip(target_speed, -MAX_SPEED, MAX_SPEED)    
    
    # PID Speed Handler
    kp, ki, kd, i_limit = pid_params        
    speed_control = PIDControl(kp, ki, kd, i_limit, MAX_PWM)
    
    ALPHA   = np.exp(-dt / tau)
    BETA    = K * (1 - ALPHA)
    
    for k in range(N):
        if (k - d - 1) < 0:
                continue
        
        setpoint[k] = target_pps if t[k] >= start_time else 0.0 # Create Step Setpoint
        
        error = setpoint[k] - y[k-1]
        u[k] = speed_control.compute(error)         # Compute PID Control
        y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]     # Difference Equation: Update Speed
```

## Verification

<table>
  <tr align = "center">
    <th  align="center" width=50>Speed (RPM)</th>
    <th  align="center">Positive Direction</th>
    <th  align="center">Negative Direction</th>
  </tr>

  <tr>
    <td align="center"> 100 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_100.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_100.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 200 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_200.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_200.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 300 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_300.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_300.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 400 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_400.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_400.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 500 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_500.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_500.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 600 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_600.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_600.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 700 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_700.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_700.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 800 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_800.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_800.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 900 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_900.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_900.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 1000 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_1000.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_1000.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 1100 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_1100.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_1100.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 1200 </td>
    <td> 
        <img src="../assets/02_Speed_Control/A_1200.jpg">
    </td>
    <td> 
        <img  src="../assets/02_Speed_Control/B_1200.jpg">
    </td>
  </tr>

</table>

#
<div align="center">
  <a href="02-System-Identification.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="04-Position-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  System Identification
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="740" height="1">
  Position Control
</div>
    
#
