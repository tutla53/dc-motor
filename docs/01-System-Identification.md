# System Identification

<div align="center">
  <a href="README.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="02-Control-Implementation.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  DC Motor System
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="700" height="1">
  Control Implementation
</div>
    
#

## Method

After creating the mathematical model of the DC motor we will try to identify the parameters of the motor. This step is very useful to understand the dynamic and the stability of the motor or even we can move further to the simulation and implementing and tuning control system. We will not identify all motor parameters like $K_t$, $K_b$, $L$, $R$, $J$, and $B$ with detail but instead we will identify them from the `first-order system` form. The parameters that we will identify are:
- Steady-state gain $(K)$
- Time-constant $(\tau)$
- Time-delay $(D)$

In this test, we will use the numerical method from python `scipy.optimize.differential_evolution` to estimates the unknown parameters simultaneously by minimizing the differences between model predictions from difference equation and the actual motor open loop response. The code below shows the setup of this test:

```python
from scipy.optimize import differential_evolution

parameters_bound = [(0.01, 0.50),   # K
                    (0.01, 0.10),   # tau
                    (0.00, 0.05)]   # D

result = differential_evolution(
    objective_function, 
    parameters_bound, 
    args=(open_loop_data, time_sampling),
    strategy='best1bin',
    tol=0.01,
    mutation=(0.5, 1),
    polish=True
)
```

For the optimization paramaters, we choose:
- `strategy='best1bin` = mutate the best candidate using one population difference; apply binomial crossover
- `tol = 0.01` = relative convergence tolerance on the population’s scores
- `mutation=(0.5, 1)` = randomly choose mutation weight $F\in[0.5,1)$ each generation
- `polish=True` = refine the best solution with bounded local optimization, normally L-BFGS-B for this call

And for the `objective_function` of this test is by minimizing the normalize `root mean squared error` (RMSE) between the model prediction and the actual motor velocity, or follows this equation: 
$$objective = \frac{\mathrm{RMSE}}{\max|y_{\mathrm{meas}}|}.$$

If the term $\max|y_{\mathrm{meas}}| = 0$, the objective value only become RMSE. The example of objective_function implementation is shown on the code below:

```python
def objective_function(self, params, open_loop_data, time_sampling):
    # Method: Root Mean Square Error (RMSE)
    
    K, tau, D = params
    
    # Filter Invalid Value
    if K <= 0 or tau <= 0 or D < 0:
        return 1e10 
    
    u, y_meas = open_loop_data
    y_sim = open_loop_response(params, u.tolist(), time_sampling)
    
    error = y_meas - y_sim
    mse = np.mean(error**2)
    rmse = np.sqrt(mse)
    
    target = np.max(np.abs(y_meas))
    if target == 0: target = 1 
    
    total_error = rmse / target
        
    return total_error    
```

And the implementation of the difference equation is shown on the code below. This implementation approximates the delay using whole samples: $(d=\lfloor D/T_s\rfloor)$. The effective simulated delay is $dT_s$.

```python
def open_loop_response(self, params, u: list, dt):        
    '''
        Input u: PWM (Ticks)
        Output y: Motor Speed (Pulse per Second)
    '''
    
    K, tau, D = params
    
    # Simulation Variable
    N     = len(u)        # Number of Data
    d     = int(D / dt)   # Time Delay
    y     = [0.0] * N     # Output: Motor Speed (Pulse per Second)
    
    # Motor Parameters
    ALPHA = np.exp(-dt / tau)
    BETA  = K * (1 - ALPHA)
    
    for k in range(N):
        if (k - d - 1) < 0:
            continue
        # Difference Equation: Update Speed
        y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]
        
    return y

```

With that system identification tools, we can estimate the motor parameters from the actual motor open loop data.

### Test Setup
- The identified model represents the response from commanded PWM to filtered measured velocity, including the motor, driver, and velocity estimator.
- System Frequency = 133 MHz
- PWM Frequency = 25 kHz
- PWM Maximum Ticks = 5319
- Motor Input = Voltage PWM in ticks
- Motor Output = Motor Velocity in pulse/s
- Velocity Sampling Period = 1 ms (1 kHz)
- Velocity Measurement = Position difference over each sample, followed by a 32-sample moving average (32 ms window), as described in [Control Implementation](02-Control-Implementation.md#velocity)
- PWM Input Test Case = −5300 to 5300, Δticks = 100

### Motor Parameters Unit
- Steady-state gain $(pulse/s)/ticks$
- Time-constant $(s)$
- Time-delay $(s)$

## Simulation Result
The graph below shows the fitted parameters across the tested PWM range. Little or no motion is observed below approximately 19% PWM magnitude. Away from this deadband and its transition, the plotted time constants ($\tau$) and delays ($D$) cluster around approximately constant levels, while the steady-state gain ($K$) varies with input. The reported summary values are $\tau = 0.0265$ s and $D = 0.014$ s; at a 1 ms sampling period, a 14 ms effective delay corresponds to 14 samples. These describe the combined response from PWM command to filtered measured velocity. Fits near the deadband may reach the parameter bounds and should not be interpreted as reliable estimates of motor dynamics. The fitted gain changes with PWM magnitude and direction, rising toward a peak in the upper input range before decreasing near full input. The following PWM-versus-speed plot illustrates the corresponding steady-state behavior.

<div align="center"> 
  <img src="../assets/01_System_Identification/System_Identification_Result.jpg" width="800"></img>
</div>

### Motor Linearity
The PWM-versus-speed plot shows a nonlinear steady-state response and different behavior in the two directions. Speed is expressed in RPM for comparison with the motor specification. The regions below are approximate descriptions of the observed motor-and-driver response, using PWM magnitude. They do not establish which physical mechanism causes each change in slope. Friction and driver behavior are possible contributors, but separating them requires additional measurements. An approximately straight steady-state curve also does not by itself establish transient-model accuracy.

<table>
  <tr align = "center">
    <th  align="center">Graph</th>
    <th  align="center" width="250">Description</th>
  </tr>

  <tr align = "center">
    <td  align="center">
      <img src="../assets/01_System_Identification/Motor_Linearity.jpg" width="800"></img>
    </td>
    <td align="left">
      <b>[1] Deadband Zone</b><br>
      At approximately 0 - 19% PWM magnitude, little or no motion is observed. This region is described as the deadband. Insufficient drive torque to overcome static friction is one possible explanation, but the speed measurements alone do not separate friction from driver or load effects.<br><br>
      <!--  -->
      <b>[2] Nonlinear Transition</b><br>
      At approximately 19 - 30% PWM magnitude, the motor begins to move and the slope of the steady-state speed curve changes. A constant-gain model may therefore fit this region poorly. Speed-dependent friction, including a possible Stribeck effect, could contribute, but this mechanism has not been isolated by these tests.<br><br>
      <!--  -->
    </td>
    <tr>
    <td colspan=2>
      <b>[3] Linear Region</b><br>
      At approximately 30 - 75% PWM magnitude, the steady-state speed curve is approximately straight. This makes the region a candidate for fitting a linear approximation and evaluating controller tuning. It does not show that friction is exclusively viscous. The approximation must still be checked against transient measurements over the intended operating range.<br><br>
      <!--  -->
      <b>[4] Pre-saturation</b><br>
      At approximately 75 - 90% PWM magnitude, the slope departs from the middle-range linear approximation before reaching the upper speed plateau. Here, pre-saturation is a descriptive label for that observed transition. The curve does not establish saturation of the back-EMF constant, H-bridge, or other components; identifying the cause would require measurements such as motor-terminal voltage, current, and temperature.<br><br>
      <!--  -->
      <b>[5] Saturation</b><br>
      At approximately 90 - 100% PWM magnitude, further command increases produce little change in the measured steady-state speed. This is an observed speed plateau under the test conditions; it does not identify which component or limitation causes the plateau.
    </td>
    </tr>
  </tr>  
</table>

<br>Based on the linearity of the motor, we can get the motor zone based on the PWM and RPM on the table below:

<div align="center">
  <table>
    <tr>
      <th rowspan=2 width=200>Zone</th>
      <th align="center" colspan=2 width=150>PWM (%)</th>
      <th align="center" colspan=2 width=150>RPM</th>
      <th rowspan=2>Criterion</th>
    </tr>
    <tr>
      <th>Min</th>
      <th>Max</th>
      <th>Min</th>
      <th>Max</th>    
    </tr>
    <tr>
      <td>Deadband</td>
      <td>0</td>
      <td>19</td>
      <td>0</td>
      <td>0</td>
      <td>Motor Speed below 15 RPM</td>
    </tr>
    <tr>
      <td>Nonlinear Transition</td>
      <td>19</td>
      <td>30</td>
      <td>0</td>
      <td>450</td>
      <td></td>
    </tr>
    <tr>
      <td>Linear</td>
      <td>30</td>
      <td>75</td>
      <td>450</td>
      <td>1100</td>
      <td>Steady-state Gain (K) changes is below 2.5%</td>      
    </tr>
    <tr>
      <td>Pre-saturation</td>
      <td>75</td>
      <td>90</td>
      <td>1100</td>
      <td>1470</td>
      <td></td>
    </tr>
    <tr>
      <td>Saturation</td>
      <td>90</td>
      <td>100</td>
      <td>1470</td>
      <td>1470</td>
      <td>Speed changes is below 2% </td>
    </tr>            
  </table>
</div>

The table below shows the summary of the system identification process:

<div align="center">
  <table>
    <tr>
      <th rowspan=2 width=150>Parameters</th>
      <th rowspan=2 width=50>Symbol</th>
      <th align="center" colspan=2 width=200>Linear Model</th>
      <th align="center" rowspan=2 width=200>Nonlinear Model</th>
    </tr>
    <tr>
      <th>Positive Direction</th>
      <th>Negative Direction</th>
    </tr>
    <tr>
      <td>Steady-state Gain (pps/ticks)</td>
      <td align="center">K</td>
      <td align="center">0.2059</td>
      <td align="center">0.1963</td>
      <td align="center">Interpolated from System Identification Result</td>       
    </tr>
    <tr>
      <td>Time-constant (s)</td>
      <td align="center">τ</td>
      <td align="center" colspan=3> 0.0265 </td>
    </tr>
    <tr>
      <td>Time-delay (s)</td>
      <td align="center">D</td>
      <td align="center" colspan=3> 0.014 </td>        
    </tr>    
  </table>
</div>
- Notes: pps = pulse per seconds

## Nonlinear Simulation Model
The nonlinear model interpolates the fitted gain as a function of signed PWM input. This allows the model to represent the observed variation in steady-state response while retaining the chosen time constant and delay. Interpolation adds a calculation to each simulation update, but its execution-time cost has not been measured here. Accuracy between measured operating points, during reversals, and under different loads requires validation; interpolation alone does not establish accuracy across the full input range.

```python    
for k in range(N):
    if (k - d - 1) < 0:
        continue
    
    # Update Motor Gain based on the PWM Input
    K_intrp = interpolate(K_LIST, u[k-d-1])
    BETA    = K_intrp * (1 - ALPHA)
    
    # Difference Equation: Update Speed
    y[k] = ALPHA * y[k-1] + BETA * u[k-d-1]

```

## Verification

The table below compares measured open-loop responses with the linear and nonlinear simulations at selected positive PWM levels. The displayed curves show improved agreement for the nonlinear model, particularly where a constant-gain model misses the steady-state speed. These comparisons provide qualitative evidence for the displayed cases; they do not establish a numerical accuracy level, performance in both directions, or accuracy throughout the full input range.

This document does not establish whether the comparison logs were excluded from parameter fitting and gain-table construction. Until their provenance is recorded, these plots should be treated as response comparisons rather than independent predictive validation. A quantitative validation should identify the fitting and held-out datasets, include both directions, and report errors for each case, such as RMSE in pulse/s and steady-state speed error. Peak-normalized RMSE can also be reported for nonzero responses, with the stated zero-speed fallback distinguished from a relative error. No such validation metrics are reported here.

<div align="center">
  <table>
    <tr align = "center">
      <th  align="center" width=50>PWM Input (%)</th>
      <th  align="center" width=500>Linear Model</th>
      <th  align="center"width=500>Nonlinear</th>
    </tr>
    <tr>
      <td align="center"> Deadband (9%) </td>
      <td> 
          <img src="../assets/01_System_Identification/verification/Linear/A_9.jpg">
      </td>
      <td> 
          <img src="../assets/01_System_Identification/verification/Nonlinear/A_9.jpg">
      </td>
    </tr>
    <tr>
      <td align="center"> Nonlinear Transition (24%) </td>
      <td> 
          <img src="../assets/01_System_Identification/verification/Linear/A_24.jpg">
      </td>
      <td> 
          <img src="../assets/01_System_Identification/verification/Nonlinear/A_24.jpg">
      </td>
    </tr>
    <tr>
      <td align="center" rowspan=2> Linear </td>
      <td><img src="../assets/01_System_Identification/verification/Linear/A_51.jpg"></td>
      <td><img src="../assets/01_System_Identification/verification/Nonlinear/A_51.jpg"></td>
    </tr>
    <tr>
      <td><img src="../assets/01_System_Identification/verification/Linear/A_70.jpg"></td>
      <td><img src="../assets/01_System_Identification/verification/Nonlinear/A_70.jpg"></td>
    </tr> 
    <tr>
      <td align="center"> Pre-saturation (81%) </td>
      <td> 
          <img src="../assets/01_System_Identification/verification/Linear/A_81.jpg">
      </td>
      <td> 
          <img src="../assets/01_System_Identification/verification/Nonlinear/A_81.jpg">
      </td>
    </tr>
    <tr>
      <td align="center" rowspan=2> Saturation </td>
      <td><img src="../assets/01_System_Identification/verification/Linear/A_90.jpg"></td>
      <td><img src="../assets/01_System_Identification/verification/Nonlinear/A_90.jpg"></td>
    </tr>
    <tr>
      <td><img src="../assets/01_System_Identification/verification/Linear/A_98.jpg"></td>
      <td><img src="../assets/01_System_Identification/verification/Nonlinear/A_98.jpg"></td>
    </tr>
  </table>
</div>

<div align="center">
  <a href="README.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="02-Control-Implementation.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  DC Motor System
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="700" height="1">
  Control Implemenation
</div>
    
#
