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

## Time-Sampling
One of the most important parameter on the discrete-time system is time-sampling or sampling frequency. Time sampling is a fundamental process in discrete systems, playing a critical role in converting continuous signals into discrete signals. This conversion is essential for various applications, particularly in digital control and signal processing. On most application we need to make sure the time-sampling is constant to represent it's analog model. To determine the time-sampling we usually refer to `Nyquist-Shannon Sampling Theorem`. The Nyquist-Shannon sampling theorem states that a continuous signal can be perfectly reconstructed from its samples if it is sampled at a rate greater than twice its highest frequency. We can write that the sampling frequency ($f_s$) must be greater or equal to two times the system's bandwidth or mathematically we can write: $f_s \geq 2 \cdot f_{bandwidth}$ . This requirement is to make sure that the signal that we process is correct for all frequency and avoid aliasing.

Even though the minimum frequency must be greater or equal to 2, but in practical application we usually  set the minimum value to higher rate (e.g. 10 to 100 times). Based on our discussion on the transfer function, the DC motor bandwidth can be calculated by the equation below:
$$ f_{bandwidth} = \frac{1}{2\pi \cdot \tau } (Hz)$$

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
### Speed Control
### Position Control
#### Step Motion Profile
#### Trapezoidal Motion Profile
## Firmware Logger Implementation

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
