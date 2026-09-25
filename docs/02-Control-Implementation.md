# Control System Implementation on RP2040

<div align="center">
  <a href="01-System-Identification.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="03-Speed-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  System Identification
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  Speed Control
</div>
    
#

## Highlight
<div align="center">
	<table>
		<tr> 
			<th width=200 align="center"> Parameter</th>
			<th width=600 align="center"> Value </th>
		</tr>
		<tr> 
      <td align="left"> Time-Constant</td>
      <td align="left"> 0.0265 s (from system identification)</td>
    </tr>     
		<tr> 
      <td align="left"> Motor Bandwidth</td>
      <td align="left"> 6.01 Hz</td>
    </tr>
		<tr> 
      <td align="left"> Control Sampling</td>
      <td align="left"> 
        Sampling Frequency: 1000 Hz <br>
        Time-Sampling: 1 ms
      </td>
    </tr>    
		<tr>  
		<tr> 
      <td align="left"> Main Control Loop Sampling Method</td>
      <td align="left"> Periodic asynchronous task using
        <a href="https://docs.embassy.dev/embassy-time/0.5.1/default/struct.Ticker.html"><code>embassy-time::Ticker</code></a>
      </td>
    </tr>     
		<tr> 
      <td align="left"> Position Measurement Method</td>
      <td align="left">
        <ul>
          <li>Implemented with 
          <a href ="https://docs.embassy.dev/embassy-rp/0.10.0/rp2040/pio_programs/rotary_encoder/struct.PioEncoder.html"><code>PioEncoder</code></a>
          to read the encoder position and direction </li>
          <li> Create a task to keep the position pulse only </li>
        </ul> 
      </td>
    </tr>
		<tr> 
      <td align="left"> Speed Measurement Method</td>
      <td align="left">
        <ul>
          <li>Get the position at the beginning of the control loop to get the delta position</li>
          <li>Update the current speed by using <code>moving average filter</code> with 32-sample delta position window</li>
        </ul> 
      </td>
    </tr>
		<tr> 
      <td align="left"> Handling Floating-Point Data Type</td>
      <td align="left"> Fixed-point arithmetic to represent the "float" as an integer by using
        <a href="https://crates.io/crates/fixed"><code>fixed</code> </a> crate
      </td>
    </tr>                              
	</table>
</div>

## Time-Sampling
One of the most important parameter on the discrete-time system is time-sampling or sampling frequency. Time sampling is a fundamental process in discrete systems, usually a `constant-value`, playing a critical role in converting continuous signals into discrete signals. This conversion is essential for various applications, particularly in digital control and signal processing. Ideally if we don't have any limit resources we can choose very high sampling frequency to represent the analog signal. But because we have some limited resources (e.g. computational limit) we need to design the proper time sampling without losing any information from the analog system. There are several requirements in designing the time-sampling such as:
- Nyquist-Shannon Sampling Theorem
- Samples per Time Constant

### Nyquist-Shannon Sampling Theorem
The Nyquist-Shannon sampling theorem states that for a band-limited measured signal whose highest frequency is $f_\text{max}$, Nyquist–Shannon requires $f_s > 2 \cdot f_\text{max}$. This requirement is to make sure that the signal that we process is correct for all frequency and avoid aliasing. As a starting point, idealized calculation, we can assume that the measured speed signal is band-limited to the motor bandwidth. The DC motor bandwidth refers to how quickly the motor can respond to changes in the input command, in this case the voltage input. 

Please take a note that the motor’s bandwidth alone does not establish this signal bandlimit. 

If we refer to the DC motor first-order voltage to velocity transfer function model, the motor bandwidth can be calculated by using this formula:

$$ f_{bandwidth} = \frac{1}{2 \pi \tau } (Hz)$$

Based on the system identification result the time-constant of the motor is `0.0265 s`, so the DC motor bandwidth is `6.01 Hz`. 
By that calculation, Nyquist–Shannon requires $f_s > 12.02$ Hz or:

$$T_s < 83.19 \text{ ms}$$

This is a `theoretical reconstruction limit`, not a recommended controller sampling period.

### Samples per Time Constant
In the first order system, time-constant ($\tau$) is refer to the time for a system step response to reach 63.2% of it's steady-state value. This value also important to predict when the DC motor reach steady state, for 95% it will require $3\tau$ and for 98% it will require $4\tau$. In this DC motor case, for a step voltage input, the motor will reach 63.2% of it's steady-state speed in 26.5 ms after the identified delay. For this design, we choose at least ten controller updates per time constant as an initial time-resolution criterion:

$$T_s \leq 2.65 \text{ ms}$$

### Selecting Time Sampling
For more safety and there's a room for on the RP2040 I choose the time sampling of `1 ms` or the sampling frequency of `1 kHz`. It satisfies the Nyquist requirement under the signal-bandlimit assumption stated above. We can estimate the additional phase lag under the following timing assumption. At a 1 ms sampling period, assuming an effective sampling/computation delay of $1.5T_s$, the additional phase lag at 6.01 Hz is approximately $3.25 \degree$. This satisfies a chosen $10 \degree$ budget for that contribution. Complete control-loop assessment must also include speed-estimator dynamics and actual update latency, evaluated at the controller’s gain crossover frequency. 

Reference: [control-delay criterion](https://imperix.com/doc/help/discrete-control-delay)

### Implementation

The discrete-time controller is designed using a constant sampling period ($T_s$). The implementation should therefore keep actual sampling intervals close to this period and limit the delay between reading feedback and applying the calculated output. Excessive timing variation can change the controller response relative to the discrete-time model. A hardware-timer interrupt like `Interrupt Service Routine (ISR)` is one option for scheduling control updates with low latency. However, its timing still depends on interrupt priorities, interrupt masking, and execution time. For this implementation, the control loop uses `embassy-time::Ticker` with a period of 1 ms, corresponding to a nominal sampling frequency of 1 kHz. This allows the control algorithm to execute as an asynchronous task alongside encoder servicing and the second motor controller.

The following simplified example shows the scheduling structure. The complete code can be found on: [`firmware/main/src/tasks/dc_motor.rs`](../firmware/main/src/tasks/dc_motor.rs)

```Rust
// PID Control Loop Structure

use embassy_time::{Duration, Ticker};

const TIME_SAMPLING_US: u64 = 1000;

#[embassy_executor::task]
async fn run_motor_task() {
    let mut ticker = Ticker::every(Duration::from_micros(TIME_SAMPLING_US));

    loop {
        ticker.next().await;

        // Read encoder feedback.
        // Estimate speed and calculate PID output.
        // Update PWM.    
    }
}
```

The ticker maintains a periodic schedule rather than starting a new 1 ms delay after each calculation (e.g. `delay()` on Arduino). While the next tick is pending, the executor can service other ready tasks without busy waiting. This provides a straightforward way to coordinate periodic control and asynchronous encoder processing. The 1 ms period is a scheduling target, not an automatic execution guarantee. Other tasks, interrupts, resource waits, or flash operations can delay an update. The complete control path must therefore be assessed for maximum scheduling delay, execution time, and missed deadlines. Overdue ticks may complete immediately, so the implementation also needs an explicit policy for missed updates rather than assuming that every iteration represents a normally spaced sample.

Reference: [Embassy Ticker](https://docs.embassy.dev/embassy-time/0.5.1/default/struct.Ticker.html)

## Encoder Reading Method
### Position 
We can measured the motor position by counting how much rotary encoder signal or pulse. We can convert the pulse to the other unit like angle or distance unit. This measurement method could be very instensive especially on high speed DC motor. For example, on our DC motor we have:
- Rotary Encoder = 48.4 pulse/rotation
- Maximum Physical Speed = 1500 RPM or 25 rotation/seconds
- Rotary Encoder Maximum Frequency = `1210 pulse/seconds`

To accomodate that, we can create a task just to keep counting for every position change. On the `embassy-rp` we can use the `PioEncoder` to read the rotary encoder position for both clockwise and counter clockwise direction. This will be useful to reduce the CPU load during reading two ditial pin of the rotary encoder. The example of the PioEncoder can be found [here](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/pio_rotary_encoder.rs). To avoid the data racing during read and write the current position, we can use `Atomic`, specifically for this project we use `AtomicI32`, which can be safely shared between threads (control, logger, etc). The code below shows the example how to updating the motor position by using AtomicI32 and PioEncoder. The complete code can be found on: [`firmware/main/src/tasks/dc_motor.rs`](../firmware/main/src/tasks/dc_motor.rs)


```Rust
// Measuring Motor Position from Rotary Encoder

use core::sync::atomic::AtomicI32;
use embassy_rp::pio_programs::rotary_encoder::Direction;

pub static CURRENT_POS: AtomicI32 = AtomicI32::new(0);

#[embassy_executor::task]
async fn run_encoder_task() {
    loop {
        let step = match self.encoder.read().await {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };

        let current_pos = CURRENT_POS.load(Ordering::Relaxed);

        CURRENT_POS.store(current_pos.saturating_add(step), Ordering::Relaxed);
    }
}
```

### Velocity
To measure velocity of the motor, usually we have two options: (1) measuring how many pulse at a `constant-time` interval or (2) measuring time at a `constant-pulse` interval. Based on our control loop architecture, the `constant-time` method provides a straightforward implementation because it uses the position samples already available to the periodic control task. The formula for this implementation is shown on the equation below:

$$v_{\text{raw}}[k]=\frac{p[k]-p[k-1]}{T_s}$$

But the main problem with measuring pulse at a constant-time interval is a shorter measurement interval produces a larger velocity increment per encoder count, resulting in coarser velocity resolution. For example, with our 1 ms sampling time, each additional count changes the raw estimate by:

$$ \Delta v=\frac{1}{0.001}=1000\text{ pulse/s} $$

Consequently, a constant physical velocity can produce readings of zero or 1000 pulse/s. Let's take a example at the case of a motor running constantly at 800 pulse/seconds and the encoder reading constant-time interval is 1 ms. 

Assumptions:
- Motor is on steady-state velocity.
- Encoder count is initialized to zero at $t=0$.
- Successive encoder counts occur every 1.25 ms.
- Counts occurring exactly at a sampling boundary are included in this example. 
- The zero-time row represents initialization rather than a measured velocity.

Then the measurement reading will be resulting something like this:

<div align="center">
<table align="center">
  <thead>
    <tr><th>Time<br>(ms)</th><th>Count</th><th>Δcount</th><th>Velocity<br>(pulse/s)</th><th>Count Update<br>(ms)</th></tr>
  </thead>
  <tbody>
    <tr><td align="right">0</td><td align="right">0</td><td align="right">0</td><td align="right">0</td><td align="right">—</td></tr>
    <tr><td align="right">1</td><td align="right">0</td><td align="right">0</td><td align="right">0</td><td align="right">—</td></tr>
    <tr><td align="right">2</td><td align="right">1</td><td align="right">1</td><td align="right">1000</td><td align="right">1.25</td></tr>
    <tr><td align="right">3</td><td align="right">2</td><td align="right">1</td><td align="right">1000</td><td align="right">2.50</td></tr>
    <tr><td align="right">4</td><td align="right">3</td><td align="right">1</td><td align="right">1000</td><td align="right">3.75</td></tr>
    <tr><td align="right">5</td><td align="right">4</td><td align="right">1</td><td align="right">1000</td><td align="right">5.00</td></tr>
    <tr><td align="right">6</td><td align="right">4</td><td align="right">0</td><td align="right">0</td><td align="right">—</td></tr>
    <tr><td align="right">7</td><td align="right">5</td><td align="right">1</td><td align="right">1000</td><td align="right">6.25</td></tr>
    <tr><td align="right">8</td><td align="right">6</td><td align="right">1</td><td align="right">1000</td><td align="right">7.50</td></tr>
    <tr><td align="right">9</td><td align="right">7</td><td align="right">1</td><td align="right">1000</td><td align="right">8.75</td></tr>
    <tr><td align="right">10</td><td align="right">8</td><td align="right">1</td><td align="right">1000</td><td align="right">10.0</td></tr>
  </tbody>
</table>
</div>

To increase the resolution, we can increase the measurement interval. For instance, if we use 4 ms interval then the resolution is become 250 pulse/s and the measurement result contains three or four counts, producing 750 or 1000 pulse/s. The result is better but still not good enough to measure low velocity. The simplest solution is keep increasing the measurement interval but this method will reducing the responsiveness of the motor because it takes long time to update the velocity. Because of that we need to set some limit to improve the measurement resolution but still has good responsiveness. Let say we want to achieve the velocity resolution of `40 RPM` or around `32 pulse/second` (around 3% of maximum velocity). That means we need the measurement interval of around 32 ms.

A non-overlapping 32 ms measurement window would provide a new estimate every 32 ms. Instead, this implementation uses an overlapping window containing the latest 32 position increments and updates the estimate every 1 ms or usually called as `moving average filter`. This improves the update rate while retaining the resolution and delay associated with the longer measurement window. With 32 samples at a 1 ms sampling period, the moving average introduces 15.5 ms of additional group delay relative to the raw velocity estimates. The complete 32 ms measurement window is centered approximately 16 ms before the current update. Updating the estimate every millisecond does not remove this measurement lag. The formula of this filter is shown on the equation below:

$$ \hat v[k] =\frac{1}{N}\sum_{i=0}^{N-1}v_{\text{raw}}[k-i]$$

The implementation of the velocity calculation is shown on the listing below. The filter buffer is initially filled with zeros, so early outputs include a startup transient until 32 actual increments have been collected. Before the first update, `last_pos` must be initialized to the current encoder count to avoid treating an existing position offset as motion. The complete code can be found on: [`crates/motor-control/src/filter.rs`](../crates/motor-control/src/filter.rs)


```rust
pub struct MovingAverageFilter<const WINDOW: usize> {
    delta_buffer: [i32; WINDOW],
    delta_idx: usize,
    window_sum: i32,
    pub last_pos: i32,
}

impl<const WINDOW: usize> MovingAverageFilter<WINDOW> {
    pub fn new() -> Self {
        Self {
            delta_buffer: [0; WINDOW],
            delta_idx: 0,
            window_sum: 0,
            last_pos: 0,
        }
    }

    pub fn calculate_speed(&mut self, new_pos: i32) -> I16F16 {
        let delta_pos = new_pos - self.last_pos;
        self.last_pos = new_pos;

        // Moving Average Method
        self.window_sum -= self.delta_buffer[self.delta_idx];
        self.window_sum += delta_pos;
        self.delta_buffer[self.delta_idx] = delta_pos;
        self.delta_idx = (self.delta_idx + 1) % WINDOW;

        I16F16::from_num(self.window_sum)
    }
}
```

The filter returns the accumulated position change over the measurement window. The motor task converts this count to pulses per second by multiplying it by the precomputed factor $1/(T_s \cdot N)$ like in this code example below:

```rust
// On the config
pub const TIME_SAMPLING_US: u64 = 1000; 
pub const SPEED_FILTER_WINDOW: usize = 1 << 5;
pub const TICKS_TO_PPS_PER_WINDOWS: f32 =
    1_000_000.0_f32 / (TIME_SAMPLING_US as f32 * SPEED_FILTER_WINDOW as f32);

// On the Control Loop
let ticks_to_pps_per_windows = I16F16::from_num(TICKS_TO_PPS_PER_WINDOWS);
let current_pos_ticks = self.motor.get_current_pos();
let current_speed_ticks = self.filter.calculate_speed(current_pos_ticks);

// Updating Motor Status
self.current_speed_pps_fixed = current_speed_ticks * ticks_to_pps_per_windows;
```

## PID Control
### Fixed-Point Arithmetic

The RP2040 PID controller operates at a **1 kHz sampling frequency**, providing **1 ms between control updates**. On calculating PID, we will face intensive floating-point calculation that can create the computational overhead. To reduce computational overhead, the controller uses fixed-point arithmetic through Rust’s [`fixed`](https://crates.io/crates/fixed) crate instead of the RP2040 floating-point library on `bootROM`. Fixed-point represents fractional values as scaled integers, avoiding the exponent alignment and normalization required by software floating-point arithmetic. The speed controller uses `I16F16`, a 32-bit format with 16 fractional bits and a constant resolution of approximately $0.00001526$. As a reference baseline, a [Cornell RP2040 benchmark](https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_fixed_pt/index_fixed.html), running at **125 MHz** with **`-Ofast`** optimization, measured **138 µs for floating-point versus 40 µs for equivalent-format fixed-point** over 100 multiply-and-add iterations, including loop overhead. This represents approximately **3.4× faster execution**, or a **71% reduction in execution time** for that workload. If performed once per 1 ms control period, the benchmark workload would consume **13.8% versus 4.0% of one core’s processing time**, freeing 98 µs for other work. This illustrates how lower arithmetic overhead can increase the time available for sensor processing, filtering, and PWM updates. However, Cornell’s handwritten C benchmark is not a measurement of this Rust PID controller, which includes saturating arithmetic and additional control logic. The actual benefit must therefore be measured using the compiled implementation, while verifying that numerical range, quantization, and complete-loop execution time satisfy the controller’s requirements.

### PID Implementation
The implementation of the PID by using the fixed-point arithmetic is shown on the code below. The PIDControl is designed to a generic fixed point because the `fixed` type of the speed and position control is different. Because the input range of speed control is approximately from ±1200 pulse/s, it's enough using the `I16F16` (32-bit fixed-point numbers) which has the range from $-32768$ to $32768-2^{-16}$ to accommodates the expected speed input range. The error, accumulated error, and gain products must also remain within the representable range to avoid unintended saturation.

Position commands use signed 32-bit encoder counts. The position controller therefore uses `I32F32` (64-bit fixed-point numbers) to represent this count range while retaining fractional precision. By using this we can easily calculate the PID output by calling the `compute` function.

```Rust
pub struct PIDController<T: Fixed> {
    kp: T,
    ki: T,
    kd: T,
    i_limit: T,
    integral: T,
    prev_error: T,
    max_output: i32,
}

impl<T: Fixed + Neg<Output = T>> PIDController<T> {
    #[inline(always)]
    pub fn compute(&mut self, command: i32, feedback: T) -> i32 {
        let error = T::from_num(command).saturating_sub(feedback);

        let next_integral = self.integral.saturating_add(error);
        self.integral = next_integral.clamp(-self.i_limit, self.i_limit);

        let derivative = error.saturating_sub(self.prev_error);
        self.prev_error = error;

        let p = self.kp.saturating_mul(error);
        let i = self.ki.saturating_mul(self.integral);
        let d = self.kd.saturating_mul(derivative);

        let sig = p.saturating_add(i).saturating_add(d);

        sig.to_num::<i32>().clamp(-self.max_output, self.max_output)
    }
}
```
The controller uses per-sample PID gains. Each update accumulates the error directly and calculates the derivative as the difference between the current and previous errors, without explicitly multiplying or dividing by the sampling period $T_s$. Therefore, when converting continuous-time parallel PID gains $K_p$, $K_i$, and $K_d$ to this implementation, use:
$$
\texttt{kp}=K_p,\qquad
\texttt{ki}=K_iT_s,\qquad
\texttt{kd}=\frac{K_d}{T_s}
$$

Here, $T_s$ is expressed in seconds. At the nominal sampling period of $1\,\text{ms}$, these become $\texttt{ki}=0.001K_i$ and $\texttt{kd}=1000K_d$. The implementation stores and uses these discrete gains directly; it does not perform this conversion internally. Gains already tuned using this implementation should not be converted again. If the sampling period changes, the gains must be reconsidered to preserve the intended controller response.

<!-- ### Speed Control
### Position Control
#### Step Motion Profile
#### Trapezoidal Motion Profile
## Firmware Logger Implementation -->

#
<div align="center">
  <a href="01-System-Identification.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="03-Speed-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  System Identification
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  Speed Control
</div>
    
#
