<!-- LaTex Definition -->
$$\def\Tall{\mathcal{Z} \left\{ \frac{G(s)}{s} \right\}}$$

# DC Motor System Identification


In order to optimize the parameter control of DC motor, we need the mathematical model to simulate the DC motor behaviour. The DC Motor block diagram in this project is shown on the picture below:

$$
    \xrightarrow{\substack{\text{Input} \\ \text{PWM}}}
    \boxed{\quad \vphantom{\Tall} \text{H-Bridge} \quad}
    \xrightarrow{\substack{\text{Output} \\ \text{Voltage}}}
    \boxed{\quad \vphantom{\Tall} \text{DC Motor} \quad}
    \xrightarrow{\substack{\text{Motor} \\ \text{Speed}}}
    \boxed{\quad \vphantom{\Tall} \text{Gearbox} \quad}
    \xrightarrow{\substack{\text{Output} \\ \text{Speed}}}
$$

In this project we only interest with the dynamic response between the input PWM and output speed. Because of that, we can say that the system that we want to identify is the H-Bridge, DC Motor, and Gearbox as one system. The picture below shows the block diagram between the input and the output that we want to identify.

$$
    \xrightarrow{\substack{\text{Input} \\ \text{PWM}}}
    \boxed{\quad \vphantom{\Tall} G(s) \quad}
    \xrightarrow{\substack{\text{Output} \\ \text{Speed}}}
$$

If we derive the mathematical model from the mechanical and electrical system we can get the `second-order system` model as shown on the equation below:

$$ 
    G(s) = \frac{K_t}{(Ls + R)(Js + B) + K_t K_b}  \left[ \frac{rad/s}{V}\right]
$$
where:
* $K_t$ : Torque Constant $\left( \frac{N \cdot m}{A} \right)$
* $K_b$ : Back EMF Constant $\left( \frac{rad/s}{V} \right)$
* $L$ : Motor Inductance $(H)$
* $R$ : Motor Resistance $(\Omega)$
* $J$ : Rotor Inertia $(kg/m^2)$
* $B$ : Viscous Friction Constant $(N \cdot m \cdot s)$

Depends on the aplication, if we need to accurately model the DC Motor then we need to get all of those paraemters. There's several ways to get those DC Motor Parameter:
* Ideally, if the DC motor manufacturer list all of those parameters then we can easily model the DC motor. 
* On the other hand if we cannot get the specification, we can measure all the motor parameters experimentally one by one. But please take a not that not all parameters is easy to measure, especially for $J$ and $B$ because they are dynamic.  
* System identification by collecting the open-loop response between output speed and input PWM then perform numerical-method optimization to estimate the mathematical equation that represent the open-loop data.


In this project we will not measure all parameters because we dont need to identify the parameters one by one. Because of that, we will perform the system identification by using the optimization process.


## DC Motor Model
### Model Modification
Based on the discussion above the DC Motor can be described as the `second-order system` which generated from 6 parameters. We can reduced the system order by eliminating the least significant parameters. If we look back to the second-order model we found that we have two types of time constant: (1) electrical time-constant $\tau_e = L/R$ and (2) mechanical time-constant $\tau_m = J/B$. Typically the ratio between $\tau_m$ and $\tau_e$ on DC Motor with gearbox could be 100 to more than 1000 because the $L$ is usually very small an the $J$ is getting higher with the gearbox.

Because of that we can reducing the order to the `first-order system` to form this equation:

$$ 
    G(s) = \frac{K_t}{R(Js + B) + K_t K_b} = \frac{K_t}{RJs + (RB + K_t K_b)} = \frac{\frac{K_t}{(RB + K_t K_b)} }{\frac{RJ}{(RB + K_t K_b)}s + 1}
$$

Or we can simplify the equation as shown below:

$$ 
    G(s) = \frac{K}{\tau s + 1}
$$

where:
* $G(s)$ : transfer function in the Laplace domain
* $K$ : static gain
* $\tau$ : time constant

The last piece that we need to consider is in reality we often face delayed response of DC motor. That could be from the firmware delay or hardware delay. Because of that we can add the time delay parameter to the transfer function as shown on the equation below.

$$ 
    \boxed {\quad G(s) = \left( \frac{K}{\tau s + 1} \right) e^{-Ls} \quad}
$$

where:
* $G(s)$ : transfer function in the Laplace domain
* $K$ : static gain
* $\tau$ : time constant
* $L$ : time delay

By using that model we can identify the DC Motor by only three parameters. This is the common model that usually used to identify the DC Motor in real life.

### Discrete-Time System Model
To works with the embedded system, we need to works in the discrete system. One method that usually usefull to transform the transfer function to the discrete-time transfer function $G(z)$ is by using the Zero-order Hold (ZOH) Method and then transform it to the difference equation. To do that, we can use this equation to transform the $G(s)$ to $G(z)$.

$$
    G(z) = (1 - z^{-1}) \cdot \mathcal{Z} \left\{ \frac{G(s)}{s} \right\}
$$

This graph below shows the detail step to get the difference equation.

$$ 
    % Transfer Function
    \underset{\substack{\text{Transfer} \\ \text{Fucntion}}}{\boxed{\quad \vphantom{\Tall} G(s) \quad}} 
    \xrightarrow{\hspace{0.2cm}}   

    % Refactoring the Transfer Function
    \underset{\substack{\text{Refactoring} \\ \text{Transfer} \\ \text{Fucntion}}}
    {\boxed{\quad \vphantom{\Tall} \frac {G(s)}{s} \quad }} 
    \xrightarrow{\hspace{0.2cm}} 

    % Applying Z-transform
    \underset{\text{Z-Transform}}{\boxed{\vphantom{\Tall} \mathcal{Z} \left\{ \frac{G(s)}{s} \right\}}} 
    \xrightarrow{\hspace{0.2cm}} 

    % Appling ZOH
    \underset{\substack{\text{Applying} \\ \text{ZOH}}}{\boxed{\quad \vphantom{\Tall} G(z) \quad}}
    \xrightarrow{\hspace{0.2cm}} 

    % Appling ZOH
    \underset{\substack{\text{Find the Output} \\ \text{Discrete Function}}}{\boxed{\vphantom{\Tall} Y(z) = G(z) \cdot U(z)}} 
    \xrightarrow{\hspace{0.2cm}} 

    % Difference Equation
    \underset{\substack{\text{Convert to} \\ \text{Difference Eq.}}}{\boxed{\quad \vphantom{\Tall} y[n] \quad}} 
$$

After the ZOH discretization process above the difference equation is shown in the equation below:

$$
    \boxed {\vphantom{\Tall} \quad y[n] = \alpha \cdot y[n-1] + \beta \cdot u[n-d-1] \quad} 
$$

where:
* $ \alpha = e^{-L/\tau}$ 
* $ \beta = K \cdot (1 - \alpha) $

By using that difference equation we can construct the algorithm to simulate the DC Motor. The remaining problem is we need to characterize the value of $K$, $\tau$, and $L$. The plan is we need to collect the open-loop data from the real DC Motor and perform numerical-method to optimize the value the DC Motor parameter.

## System Identification Algorithm
### Method
### Result
## Verification