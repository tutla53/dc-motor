# Control System Implementation on RP2040

<div align="center">
  <a href="README.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="02-System-Identification.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  DC Motor System
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  System Identification
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
      <td align="left"> Motor Bandwidth (est.)</td>
      <td align="left"> 6.2 Hz</td>
    </tr>
		<tr> 
      <td align="left"> Time-Constant (est.)</td>
      <td align="left"> 0.025 - 0.045 s</td>
    </tr> 
		<tr> 
      <td align="left"> Control Sampling</td>
      <td align="left"> 
        Sampling Frequency: 200 Hz (~62 times motor bandwidth) <br>
        Time-Sampling: 5 ms
      </td>
    </tr>    
		<tr>  
		<tr> 
      <td align="left"> Main Control Loop Sampling Method</td>
      <td align="left"> Pooling with 
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
          <li> Create a task to keep the position counter only </li>
        </ul> 
      </td>
    </tr>
		<tr> 
      <td align="left"> Speed Measurement Method</td>
      <td align="left">
        <ul>
          <li>Get the position at the beginning of the control loop to get the delta position</li>
          <li>speed = delta_position / time_sampling</li>
        </ul> 
      </td>
    </tr>
		<tr> 
      <td align="left"> Handling Floating-Point Data Type</td>
      <td align="left"> Fixed-point arithmetic to represent the "float" as an integer by using
        <a href="https://crates.io/crates/fixed"><code>fixed</code> </a> crate
      </td>
    </tr>
		<tr> 
      <td align="left"> Inter-Task and Inter-Core Communication Method</td>
      <td align="left">
        <a href="https://rust.docs.kernel.org/6.1/core/sync/atomic/struct.AtomicI32.html"><code>AtomicI32</code></a>
        : Transfer single interger data (i32) to control tasks safely without blocking
        <ul>
          <li>Current Position Data</li>
          <li>Current Speed Data</li>
          <li>Current Commanded Position Data</li>
          <li>Current Commanded Speed Data</li>
          <li>Current Commanded PWM Data</li>
        </ul>       
        <a href="https://docs.embassy.dev/embassy-sync/git/default/channel/index.html"><code>embassy_sync::channel</code></a>
        : Transfer time-sensitve data (critical) with struct data type and interact with the control task and usb communication task
        <ul>
          <li>Event Data</li>
          <li>Command Data</li>
          <li>Firmware Logger Data</li>
          <li>Motor Command Data</li>          
        </ul>
        </ul>
        <a href="https://docs.embassy.dev/embassy-sync/git/default/mutex/index.html"><code>embassy_sync::mutex</code></a>
        : Transfer non time-sensitve config with struct data type
        <ul>
          <li>Position Control PID Config</li>
          <li>Speed Control PID Config</li>
        </ul>         
      </td>
    </tr> 
		<tr> 
      <td align="left"> Task List</td>
      <td align="left">
        CORE 0
        <ul>
          <li><code>usb_device_task</code> : Start USB Communication</li>
          <li><code>usb_rx_task</code> : Handling Received Message</li>
          <li><code>usb_tx_task</code> : Handling Firmware Response</li>
          <li><code>firmware_logger_task</code> : Handling Data Streaming</li>
          <li><code>heartbeat_task</code> : LED Indicator</li>
          <br>
        </ul>
        CORE 1
        <ul>
          <li><code>motor0_task</code> : Motor 0 main control loop</li>
          <li><code>encoder0_task</code> : Rotary Encoder Counter for Motor 0</li>
          <li><code>motor1_task</code> : Motor 1 main control loop</li>
          <li><code>encoder1_task</code> : Rotary Encoder Counter for Motor 1</li>      
        </ul>
      </td>
    </tr>                                
	</table>
</div>

## Time-Sampling
One of the most important parameter on the discrete-time system is time-sampling or sampling frequency. Time sampling is a fundamental process in discrete systems, playing a critical role in converting continuous signals into discrete signals. This conversion is essential for various applications, particularly in digital control and signal processing. On most application we need to make sure the time-sampling is constant to represent it's analog model. To determine the time-sampling we usually refer to `Nyquist-Shannon Sampling Theorem`. The Nyquist-Shannon sampling theorem states that a continuous signal can be perfectly reconstructed from its samples if it is sampled at a rate greater than twice its highest frequency. We can write that the sampling frequency ($f_s$) must be greater or equal to two times the system's bandwidth or mathematically we can write: $f_s \geq 2 \cdot f_{bandwidth}$ . This requirement is to make sure that the signal that we process is correct for all frequency and avoid aliasing.

Even though the minimum frequency must be greater or equal to 2, but in practical application we usually  set the minimum value to higher rate (e.g. 10 to 100 times). Based on our discussion on the transfer function, the DC motor bandwidth can be calculated by the equation below:

$$ f_{bandwidth} = \frac{1}{2 \pi \tau } (Hz)$$

Because we don't know exactly the time-constant for our DC motor, we can perform open loop response test with step input and measure the settling-time. We can estimate the time-constant by $\tau = t_s/4$ for 5% settling criterion or $\tau = t_s/5$ for 2% settling criterion. For this project DC motor we got the time-constant is around 0.025 to 0.04 seconds. By using the formula above we can get the bandwidth frequency of this project DC motor is `6.4 Hz`. 

After we got the estimation bandwidth of the DC motor, we can decided how much the time-sampling that we want to apply for the firmware controller. For this project I decided to use `200 Hz` sampling frequency because it's not consuming much CPU usage with good safety factor (around `31 times` the DC motor bandwidth). Another method that we can use is how much data to reach the time-constant. For the 0.025 of time-constant, we can get 5 data with 200 Hz time-sampling. This value is enough for experimental purpose. For more critical purpose we can increase the sampling frequency to 1 kHz so we can get 25 data to time-constant. We will try to control with 1 kHz later, but for now we will stick to 200 Hz. After we get the time-sampling, we can design the firmware based on this value.

### Implementation

Based on the discussion about the discrete-time transfer function, one of the most important implementation for control system is the stability of the time-sampling. During the discretization process we assume that the $T_s$ is constant so that we can get the final model of the difference equation. If the time-sampling is not stable, the system behaviour cannot be predicted accurately. The best method to implement the constant time-sampling on the embedded system is by using the `Interrupt Service Routine (ISR)`. The ISR is relying on the hardware interrupt to call the command on the ISR function. Because of that, the latency could be very low and the time-sampling is very consistent.

But for more complex project, the ISR could be very complicated and can resulting the data racing. Because our system will use only 200 Hz of time-sampling I will use `embassy-time::Ticker` for the control loop. This Rust `embassy-rp` framework is great for the cooperative multitasking with the `async/await` method. The implementation also is quite easy as shown on the code below:

```Rust
use embassy_time::{Duration, Ticker};

const TIME_SAMPLING_US: u64 = 5000;

#[embassy_executor::task]
async fn run_motor_task() {
  let mut ticker = Ticker::every(Duration::from_micros(TIME_SAMPLING_US));

  loop {
    // Control Logic: Read Sensor, Calculate PID, Set PWM

    ticker.next().await; // Yield the task for the next tick
  }
}
```

The code above generate the `ticker` for every 5 ms (200 Hz) that will trigger the loop on the `run_motor_task`. It's not the relative delay like `delay()` on Arduino which will start the calculation after the function is called, but it's will calculate the time after declaration and will be executed with constant tick. This ticker also is not blocking delay, while the `ticker` is waiting for the next tick, the CPU will jump to another task that available. By using this method, we can create constant time-sampling for 200 Hz application.

## Encoder Reading Method
We can measured the motor position by counting how much rotary encoder signal or pulse. We can convert the pulse to radian or rotation based on the rotary encoder signal pulse per rotation. This measurement method could be very instensive especially on high speed DC motor. For example, on our DC motor we have:
- Rotary Encoder = 48.4 pulse/rotation
- Maximum Speed = 1200 RPM or 20 rotation/seconds
- Rotary Encoder Maximum Frequency = `968 pulse/seconds`

To accomodate that, we can create a task just to keep counting for every position change. On the `embassy-rp` we can use the `PioEncoder` to read the rotary encoder position for both clockwise and counter clockwise direction. This will be useful to reduce the CPU load during reading two ditial pin of the rotary encoder. The example of the PioEncoder can be found [here](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/pio_rotary_encoder.rs). To avoid the data racing during read and write the current position, we can use `Atomic`, specifically for this project we use `AtomicI32`, which can be safely shared between threads (control, logger, etc). The code below shows the example how to updating the motor position by using AtomicI32 and PioEncoder.

```Rust
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
        self.motor.set_current_pos(current_pos.saturating_add(step));
    }
}
```
We can measure the speed at the beginning of the DC Motor control loop by dividing the pos difference by the time sampling. The pos difference it self can be collected from the encoder task. By using this implementation we can perform the open loop speed control and log the position and the speed of the DC motor. The complete implementation can be found on the `firmware/main/src/tasks/dc_motor.rs`

## PID Control
### Floating Point Handler
The RP2040 doesn't have have floating-point arithmetic implemented in hardware and it is best to
avoid it if possible. The compiler like `gcc` or `rustc` comes with software floating-point libraries for processors without floating-point support. But the problem with the compiler-builtins is it takes around 60–90 clock cycles to perform a single-precision float addition or substract. Raspberry included a fast floating-point library on the `bootROM`. This library knows all the features of the RP2040 and uses the division coprocessor. This can reduce the CPU cycles 2-3 times, so this makes the floating point calculation faster. But there's faster way to handle the floating point arithmetic, is by using <a href="https://crates.io/crates/fixed"><code>fixed</code></a> which treat the float as the integer. By using that, the addition and substraction can be done by using only 1 cpu cycle. This method can speed up the floating-point arithmetic 2-3 times faster than the boot ROM and 5-6 times faster than the compiler builtins. This is very significant improvment because the PID calculation is always executed on the control loop task. 

### PID Implementation
The implementation of the PID by using the fixed-point float is shown on the code below. The PIDControl is designed to a generic fixed point because the `fixed` type of the speed and position control is different. Because the input range of speed control is from -1200 to 1200 RPM, it's enough using the `I16F16` (16-bit fixed-point numbers) which has the range from -32_768 to 32_767. For the position control it's need the `I32F32` (32-bit fixed-point numbers) because we have no limitation for the position command. By using this we can easily calculate the PID output by calling the `compute` function.

```Rust
pub struct PIDcontrol<T: Fixed> {
    kp: T,
    ki: T,
    kd: T,
    i_limit: T,
    integral: T,
    prev_error: T,
    max_threshold: i32,
}

impl<T: Fixed + Neg<Output = T>> PIDcontrol<T> {
    #[inline(always)]
    pub fn compute(&mut self, error: T) -> i32 {
        let next_integral = self.integral.saturating_add(error);
        self.integral = next_integral.clamp(-self.i_limit, self.i_limit);

        let derivative = error - self.prev_error;
        self.prev_error = error;

        let sig = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);

        sig.to_num::<i32>().clamp(-self.max_threshold, self.max_threshold)
    }
}
```

To separate the communication and the main PID control tasks, we used the multicore capability of the RP2040. CORE0 will be used for the communication tasks and CORE1 is for the control tasks.

<!-- ### Speed Control
### Position Control
#### Step Motion Profile
#### Trapezoidal Motion Profile
## Firmware Logger Implementation -->

#
<div align="center">
  <a href="README.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="02-System-Identification.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  DC Motor System
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  System Identification
</div>
    
#
