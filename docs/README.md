# DC Motor System Identification

In order to optimize the parameter control of a DC motor, we need the mathematical model to simulate the DC motor behaviour. The DC Motor block diagram in this project is shown on the picture below:

$$
    \xrightarrow{\substack{\text{Input} \\ \text{PWM}}}
    \boxed{\rule[-10pt]{0pt}{20pt}\quad \text{H-Bridge} \quad \rule{0pt}{15pt}}
    \xrightarrow{\substack{\text{Output} \\ \text{Voltage}}}
    \boxed{\rule[-10pt]{0pt}{25pt} \quad \text{DC Motor} \quad}
    \xrightarrow{\substack{\text{Motor} \\ \text{Speed}}}
    \boxed{\rule[-10pt]{0pt}{25pt} \quad \text{Gearbox} \quad}
    \xrightarrow{\substack{\text{Output} \\ \text{Speed}}}
$$

In this project, we are only interested in the dynamic response between the input PWM and output speed. We can identify the H-Bridge, DC Motor, and Gearbox as one single system:

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

## DC Motor Model
### Model Modification
We can reduce the order to a `first-order system`:

$$ 
    G(s) = \frac{K_t}{RJs + (RB + K_t K_b)} = \frac{\frac{K_t}{(RB + K_t K_b)} }{\frac{RJ}{(RB + K_t K_b)}s + 1}
$$

Simplified as:

$$ 
    G(s) = \frac{K}{\tau s + 1}
$$

Considering firmware/hardware delay, we add the time delay parameter:

$$ 
    \boxed{ \rule[-20pt]{0pt}{50pt} \quad G(s) = \left( \frac{K}{\tau s + 1} \right) e^{-Ls} \quad}
$$

### Discrete-Time System Model
Using the Zero-order Hold (ZOH) Method:

$$
    G(z) = (1 - z^{-1}) \cdot \mathcal{Z} \left[ \frac{G(s)}{s} \right]
$$

The steps to find the difference equation are visualized below:

$$ 
    \underset{\text{TF}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad G(s) \quad}} 
    \to \underset{\text{Refactoring}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad \frac{G(s)}{s} \quad}} 
    \to \underset{\text{Z-Transform}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad \mathcal{Z} \left\[ \frac{G(s)}{s} \right\] \quad}} 
    \to \underset{\text{ZOH}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad G(z) \quad}} 
    \to \underset{\text{Discrete Output}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad Y(z) = G(z) \cdot U(z) \quad}} 
    \to \underset{\text{Difference Eq.}}{\boxed{\rule[-15pt]{0pt}{35pt} \quad y[n] \quad}} 
$$

The final difference equation is:

$$
    y[n] = \alpha \cdot y[n-1] + \beta \cdot u[n-d-1]
$$

where:
* $\alpha = e^{-L/\tau}$ 
* $\beta = K \cdot (1 - \alpha)$

By using that difference equation we can construct the algorithm to simulate the DC Motor. The remaining problem is we need to characterize the value of $K$, $\tau$, and $L$. The plan is we need to collect the open-loop data from the real DC Motor and perform numerical-method to optimize the value the DC Motor parameter.

## System Identification Algorithm
### Method
### Result
## Verification
