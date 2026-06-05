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
The graph below shows the result of the system identification process. We can see that there's a deadband for the PWM below 25%. After the deadband to the maximum PWM input, we can see that the time-constant ($\tau$) and time-delay (L) has no significant changes. But for the steady-state gain (K) there's a nonlinearity behaviour based on the PWM input. After the deadband region, the value of K is increasing up to the 85% of the PWM Input, and then decreasing after that up to 100%. To analyzed further about the K, we will convert the graph from PWM vs K to PWM vs Speed.

<div align="center"> 
  <img src="../assets/01_System_Identification/System_Identification_Result.jpg" width="800"></img>
</div>

### Motor Linearity
After we conver the data to PWM vs Motor Speed, we can see that there's 5 zones that can described the DC motor behaviour.

<div align="center"> 
  <img src="../assets/01_System_Identification/Motor_Linearity.jpg" width="800"></img>
</div>

## Verification

The table below shows the comparison between the DC Motor open loop firmware log and the simulation graph.  Based on that, we can say that we have successfully created the simulation model of the DC Motor with the minimum of error that cover for both direction and various speed.

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
