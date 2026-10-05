# Control System Implementation on RP2040

<div align="center">
  <a href="02-Control-Design.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <!-- <a href="04-Position-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a> -->
</div>
<div align="center">
  Control Design
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="900" height="1">
  <!-- Position Control -->
</div>
    
#

## Overview
In this project, the control system design will be implemented on the **firmware** to `control` the motor and the **rust_script** to `simulate` the motor movement. Both of them will use the same control implementation on the `crates/motor-control/`. The complete implementation can be found on this code:
- firmware: [`firmware/main/src/tasks/dc_motor.rs`](../firmware/main/src/tasks/dc_motor.rs)
- rust_script: [`rust_script/src/simulation/model.rs`](../rust_script/src/simulation/model.rs)

<div align="center">
  <table>
    <tr>
      <th>Category</th>
      <th>Variable</th>
      <th>Description</th>
      <th>Data Type</th>
    </tr>
    <tr>
      <td rowspan=3>Motor Parameters</td>
      <td>K</td>
      <td>Static Gain</td>
      <td>f64</td>
    </tr>
    <tr>
      <td>TAU_S</td>
      <td>Time Constant</td>
      <td>f64</td>
    </tr>   
    <tr>
      <td>D_S</td>
      <td>Time Delay</td>
      <td>f64</td>
    </tr>      
    <tr>
      <td rowspan=5>Model Parameters</td>
      <td>DT_S</td>
      <td>Time Sampling</td>
      <td>f64</td>
    </tr>
    <tr>
      <td>d</td>
      <td>
        Dead-Time Index <br>
        <code>(D_S / DT_S) as usize</code>
      </td>
      <td>usize</td>
    </tr> 
    <tr>
      <td>k</td>
      <td>Time Index</td>
      <td>usize</td>
    </tr>          
    <tr>
      <td>alpha</td>
      <td>
        Discrete Pole <br>
        <code>(-DT_S / TAU_S).exp()</code>
      </td>
      <td>f64</td>      
    </tr>     
    <tr>
      <td>beta</td>
      <td>
        Input Gain <br>
        <code>K * (1.0 - alpha)</code>
      </td>
      <td>f64</td>
    </tr>         
    <tr>
      <td rowspan = 4>Simulation Variable</td>
      <td>set_point</td>
      <td>Control Desired Value</td>
      <td>Vec&lt;f64&gt;</td>
    </tr>    
    <tr>
      <td>u</td>
      <td>Array of PWM Input</td>
      <td>Vec&lt;f64&gt;</td>
    </tr>
    <tr>
      <td>y</td>
      <td>Array of Motor Speed</td>
      <td>Vec&lt;f64&gt;</td>
    </tr>
    <tr>
      <td>x</td>
      <td>Array of Motor Position</td>
      <td>Vec&lt;f64&gt;</td>
    </tr>        
    <tr>
      <td rowspan = 4>Control</td>
      <td>speed_control</td>
      <td>PID Speed Control Handler with I16F16</td>
      <td>PIDController&lt;I16F16&gt;</td>
    </tr>
    <tr>
      <td>position_control</td>
      <td>PID Position Control Handler with I32F32</td>
      <td>PIDController&lt;I32F32&gt;</td>
    </tr>
  </table>
</div>

## Motor Control on `firmware`

### Motor Command Generator
```rust
#[derive(Clone, Copy, PartialEq)]
pub enum Shape {
    Step(i32),
    Trapezoidal(I32F32, I32F32, I32F32),
}

#[derive(Clone, Copy, PartialEq)]
pub enum MotorCommand {
    SpeedControl(i32),
    PositionControl(Shape),
    OpenLoop(i32),
    Stop,
}
```


### Update Motor State

### Open Loop
```rust
MotorCommand::OpenLoop(sig) => {
    // Move Motor
    self.move_motor(sig);
}
```
### Speed Control

```rust
MotorCommand::SpeedControl(commanded_speed) => {
    // Update Commanded Speed
    MOTOR.set_commanded_speed(commanded_speed);

    // Compute PWM Output
    let commanded_speed: i32 = commanded_speed.clamp(-max_speed_pps, max_speed_pps);
    let sig = speed_control.compute(commanded_speed, current_speed_pps_fixed);

    // Move Motor
    self.move_motor(sig);
}
```

### Position Control

```rust
MotorCommand::PositionControl(input_shape) => {
    // Update Commanded Position
    let commanded_position = self.get_commanded_pos(input_shape);
    MOTOR.set_commanded_pos(commanded_position);

    // Move Done Check
    self.move_done_check(event_sender);

    // Compute Speed Output
    let target_speed = position_control.compute(commanded_position, current_pos_pulse_fixed);

    // Compute PWM Output
    let sig = speed_control.compute(target_speed, current_speed_pps_fixed);

    // Move Motor
    self.move_motor(sig);
}
```

## Motor Simulation on `rust_script`

### Open Loop Simulation
```rust
/* ---------- Open Loop ---------- */
for k in 0..u.len() {
    if (k as i32 - d as i32 - 1) < 0 {
        continue;
    }

    y[k] = alpha * y[k - 1] + beta * u[k - d - 1];
}
```

### Speed Control Simulation
```rust
/* ---------- Speed Control ---------- */
for k in 0..set_point.len() {
    if (k as i32 - d as i32 - 1) < 0 {
        continue;
    }

    let target_speed = (set_point[k] as i32).clamp(-(max_speed_pps as i32), max_speed_pps as i32);

    u[k] = speed_control.compute(target_speed, I16F16::from_num(y[k - 1])) as f64;

    y[k] = alpha * y[k - 1] + beta * u[k - d - 1];
}
```
### Position Control Simulation

```rust
/* ---------- Position Control ---------- */
for k in 0..set_point.len() {
    if (k as i32 - d as i32 - 1) < 0 {
        continue;
    }

    let target_speed = position_control.compute(set_point[k] as i32, I32F32::from_num(x[k - 1]));

    u[k] = speed_control.compute(target_speed, I16F16::from_num(y[k - 1])) as f64;

    y[k] = alpha * y[k - 1] + beta * u[k - d - 1];                    // Updating Motor Speed

    x[k] = x[k - 1] + ((y[k - 1] + y[k]) / 2.0) * motor_config::DT_S; // Update Position
}
```


## Motor Control and Simulation Result
### Speed Control Result
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

### Position Control Result

<table>
  <tr align = "center">
    <th  align="center" width=50>Position (rotation)</th>
    <th  align="center">Positive Direction</th>
    <th  align="center">Negative Direction</th>
  </tr>

  <tr>
    <td align="center"> 5 </td>
    <td> 
        <img src="../assets/03_Position_Control/A_5.jpg">
    </td>
    <td> 
        <img  src="../assets/03_Position_Control/B_5.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 50 </td>
    <td> 
        <img src="../assets/03_Position_Control/A_50.jpg">
    </td>
    <td> 
        <img  src="../assets/03_Position_Control/B_50.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 200 </td>
    <td> 
        <img src="../assets/03_Position_Control/A_200.jpg">
    </td>
    <td> 
        <img  src="../assets/03_Position_Control/B_200.jpg">
    </td>
  </tr>

  <tr>
    <td align="center"> 1000 </td>
    <td> 
        <img src="../assets/03_Position_Control/A_1000.jpg">
    </td>
    <td> 
        <img  src="../assets/03_Position_Control/B_1000.jpg">
    </td>
  </tr>

</table>

#
<div align="center">
  <a href="02-Control-Design.md"><img src="../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <!-- <a href="04-Position-Control.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a> -->
</div>
<div align="center">
  Control Design
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="900" height="1">
  <!-- Position Control -->
</div>
    
#
