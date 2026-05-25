# DC Motor System

---

<table style="width:100%; border:none; text-align:center;">
  <tr>
    <td style="text-align:left; width:40%;">
    </td>
    <td style="text-align:center; width:20%;">
      <a href="README.md">🏠 Home</a><br>
    </td>
    <td style="text-align:right; width:40%;">
      <a href="01-System-Identification.md">Next »</a><br>
      System Identification
    </td>
  </tr>
</table>

---


## Transfer Function
In order to characterize and optimize the parameter control of a DC motor, we need the mathematical model to simulate the DC motor behaviour. The DC Motor block diagram in this project is shown on the picture below:

$$
    \xrightarrow{\substack{\text{Input} \\ \text{PWM}}}
    \boxed{\rule[-10pt]{0pt}{20pt}\quad \text{H-Bridge} \quad \rule{0pt}{15pt}}
    \xrightarrow{\substack{\text{Output} \\ \text{Voltage}}}
    \boxed{\rule[-10pt]{0pt}{25pt} \quad \text{DC Motor} \quad}
    \xrightarrow{\substack{\text{Motor} \\ \text{Speed}}}
    \boxed{\rule[-10pt]{0pt}{25pt} \quad \text{Gearbox} \quad}
    \xrightarrow{\substack{\text{Output} \\ \text{Speed}}}
$$

In this project we only interest with the dynamic response between the input PWM and output speed. Because of that, the term `system` on this project is refer to the combination of H-Bridge, DC Motor, and Gearbox system. The picture below shows the block diagram between the input and the output that we want to identify.

$$
    \xrightarrow{\substack{\text{Input} \\ \text{PWM}}}
    \boxed{\rule[-10pt]{0pt}{25pt} \quad G(s) \quad}
    \xrightarrow{\substack{\text{Output} \\ \text{Speed}}}
$$

If we derive the mathematical model from the mechanical and electrical system, we get the `second-order system` model:

$$ 
    G(s) = \frac{K_t}{(Ls + R)(Js + B) + K_t K_b} \left[ \frac{\text{rad/s}}{\text{V}} \right]
$$

where:
* $K_t$ : Torque Constant $(\text{N}\cdot\text{m/A})$
* $K_b$ : Back EMF Constant $(\text{rad/s/V})$
* $L$ : Motor Inductance $(\text{H})$
* $R$ : Motor Resistance $(\Omega)$
* $J$ : Rotor Inertia $(\text{kg}\cdot\text{m}^2)$
* $B$ : Viscous Friction Constant $(\text{N}\cdot\text{m}\cdot\text{s})$

Depends on the aplication, if we need to accurately model the DC Motor then we need to get all of those paraemters. There's several ways to get those DC Motor Parameter:
* Ideally, if the DC motor manufacturer list all of those parameters then we can easily model the DC motor. 
* On the other hand if we cannot get the specification, we can measure all the motor parameters experimentally one by one. But please take a not that not all parameters is easy to measure, especially for $J$ and $B$ because they are dynamic.  
* System identification by collecting the open-loop response between output speed and input PWM then perform numerical-method optimization to estimate the mathematical equation that represent the open-loop data.


In this project we will not measure all parameters because we dont need to identify the parameters one by one. Because of that, we will perform the system identification by using the optimization process.

## DC Motor Model Modification
Based on the discussion above the DC Motor can be described as the `second-order system` which generated from 6 parameters. We can reduced the system order by eliminating the least significant parameters. If we look back to the second-order model we found that we have two types of time constant: (1) electrical time-constant $\tau_e = L/R$ and (2) mechanical time-constant $\tau_m = J/B$. Typically the ratio between $\tau_m$ and $\tau_e$ on DC Motor with gearbox could be 100 to more than 1000 because the $L$ is usually very small an the $J$ is getting higher with the gearbox.

Because of that we can reducing the order to the `first-order system` to form this equation:

$$ 
    G(s) = \frac{K_t}{RJs + (RB + K_t K_b)} = \frac{\frac{K_t}{(RB + K_t K_b)} }{\frac{RJ}{(RB + K_t K_b)}s + 1} = \frac{K}{\tau s + 1}
$$

Considering firmware/hardware delay, we add the time delay parameter:

$$ 
    \boxed{ \rule[-20pt]{0pt}{50pt} \quad G(s) = \left( \frac{K}{\tau s + 1} \right) e^{-Ls} \quad}
$$

where:
* $G(s)$ : transfer function in the Laplace domain
* $K$ : static gain
* $\tau$ : time constant
* $L$ : time delay

By using that model we can identify the DC Motor by only three parameters. This is the common model that usually used to identify the DC Motor in real life.

## Discrete-Time System Model
To works with the embedded system, we need to works in the discrete system. One method that usually usefull to transform the transfer function to the discrete-time transfer function $G(z)$ is by using the Zero-order Hold (ZOH) Method and then transform it to the difference equation. To do that, we can use this equation to transform the $G(s)$ to $G(z)$.

$$
    G(z) = (1 - z^{-1}) \cdot \mathcal{Z} \left[ \frac{G(s)}{s} \right]
$$

The steps to find the difference equation are visualized below:

$$ 
    \underset{\text{TF}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad G(s) \quad}} 
    \to \underset{\text{Refactoring}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad \frac{G(s)}{s} \quad}} 
    \to \underset{\text{Z-Transform}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad \mathcal{Z} \left[ \frac{G(s)}{s} \right] \quad}} 
    \to \underset{\text{ZOH}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad G(z) \quad}} 
    \to \underset{\text{Discrete Output}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad Y(z) = G(z) \cdot U(z) \quad}} 
    \to \underset{\text{Difference Eq.}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad y[n] \quad}} 
$$

After the ZOH discretization process above the difference equation is shown in the equation below:

$$
    y[n] = e^{-T_s/\tau} \cdot y[n-1] + K \cdot (1 - e^{-T_s/\tau}) \cdot u[n-round(\frac{L}{T_s})-1]
$$

We can simplify the equation by defining new variables: 

* $\alpha = e^{-T_s/\tau}$ : Discrete Pole
* $\beta = K \cdot (1 - \alpha)$ : Input Gain
* $d = round(L/T_s)$ : Dead-Time Index

The final equation is:

$$
    y[n] = \alpha \cdot y[n-1] + \beta \cdot u[n-d-1]
$$

where:
* $T_s$ : Time Sampling (s)
* $\tau$ : Time Constant (s)
* $K$ : System Static Gain $\left( \frac{pulse/sec}{ticks} \right)$
* $L$ : time delay (s)

condition: 
* $n > 0$
* $u= 0$ if $n < d$

Notes: The time sampling must be constant

By using that difference equation we can construct the algorithm to simulate the DC Motor. The remaining problem is we need to characterize the value of $K$, $\tau$, and $L$. The plan is we need to collect the open-loop data from the real DC Motor and perform numerical-method with three input variables to optimize the value the DC Motor parameter.

---

<table style="width:100%; border:none; text-align:center;">
  <tr>
    <td style="text-align:left; width:40%;">
    </td>
    <td style="text-align:center; width:20%;">
      <a href="README.md">🏠 Home</a><br>
    </td>
    <td style="text-align:right; width:40%;">
      <a href="01-System-Identification.md">Next »</a><br>
      System Identification
    </td>
  </tr>
</table>

---