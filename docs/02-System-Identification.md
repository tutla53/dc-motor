# System Identification

<div align="center">
  <a href="01-Control-Implementation.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="03-Speed-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  Control Implementation
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  Speed Control
</div>
    
#

## Method
### Simulation with `lfilter`

```python
import numpy as np
from scipy.signal import lfilter

def __open_loop_response(self, params, u, dt):        
    K, tau, L = params
    
    delay_samples = int(round(L / dt))
    
    # Shift the input signal u by 'delay_samples'
    if delay_samples > 0:
        u_delayed = np.zeros_like(u)
        u_delayed[delay_samples:] = u[:-delay_samples]
    else:
        u_delayed = u

    # Discrete-time coefficients
    a = np.exp(-dt / tau)
    b = K * (1 - a)
    
    # Simulate the first-order response
    y_sim = lfilter([0, b], [1, -a], u_delayed)
    return y_sim
```

## Result
### Motor Linearity
Based on the open loop log, we can get the plot between the the motor speed and the voltage PWM input as shown on the graph below. From that graph we can see that the motor starting to move with the PWM input of 19% and saturated at the 85%.

<div align="center"> 
  <img src="../assets/01_System_Identification/Motor_Linearity.jpg" width="800"></img>
</div>

## Verification

The table below shows the comparison between the DC Motor open loop firmware log and the simulation graph.  Based on that, we can say that we have successfully created the simulation model of the DC Motor with the minimum of error that cover for both direction and various speed target from 17% to 98% of PWM ticks. We can see that the motor starting to move with the PWM input of 19%.

<table>
  <tr align = "center">
    <th  align="center" width=50>PWM Input (%)</th>
    <th  align="center">Positive Direction</th>
    <th  align="center">Negative Direction</th>
  </tr>

  <tr>
    <td align="center"> 17 </td>
    <td> 
        <img src="../assets/01_System_Identification/A_17.jpg">
    </td>
    <td> 
        <img  src="../assets/01_System_Identification/B_17.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 19 </td>
    <td> 
        <img src="../assets/01_System_Identification/A_19.jpg">
    </td>
    <td> 
        <img  src="../assets/01_System_Identification/B_19.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 49 </td>
    <td> 
        <img src="../assets/01_System_Identification/A_49.jpg">
    </td>
    <td> 
        <img  src="../assets/01_System_Identification/B_49.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 85 </td>
    <td> 
        <img src="../assets/01_System_Identification/A_85.jpg">
    </td>
    <td> 
        <img  src="../assets/01_System_Identification/B_85.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 98 </td>
    <td> 
        <img src="../assets/01_System_Identification/A_98.jpg">
    </td>
    <td> 
        <img  src="../assets/01_System_Identification/B_98.jpg">
    </td>
  </tr>

</table>

<div align="center">
  <a href="01-Control-Implementation.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="03-Speed-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  Control Implementation
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  Speed Control
</div>
    
#
