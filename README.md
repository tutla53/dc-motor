# DC Motor PID Control
DC Motor Speed and Position Control with Raspberry Pi Pico RP2040 and `embassy-rs` 🦀. This is the framework to write a firmware code with communication and storage feature. In this repo we use `Python` to communicate with the firmware API via serial communication. It's possible to scale up the project with another applications.

## Features
The table below shows the firmware features:

<div align="left">
<table>
	<tr> 
		<th width = "175" align="center"> Features</th>
		<th width = "500" align="center"> Details </th>
	</tr>
	<!-- PID Motor Control -->
  <tr> 
    <td style="vertical-align:top"> PID Motor Control</td>
    <td>  
      <ul>
        <li>Speed Control &rarr; Step Motion Profile Only</li>
        <li>
          Position Control
          <ul>
          	<li>Step Motion Profile</li>
            <li>Trapezoid Motion Profile</li>
          </ul>
        </li>
				<li>Use Fixed-point Numbers Calculation via <code>fixed</code></li>
      </ul>
    </td>
  </tr>
	<!-- USB Communication -->
  <tr> 
    <td style="vertical-align:top"> Raw Byte Communication via USB CDC ACM</td>
    <td>  
      <ul>
        <li>Get and Set Firmware Config</li>
        <li>Save Firmaware Config on the Flash Memory</li>
        <li>Controlling Motor based on PID Control Mode</li>
        <li>Firmware Logger up to 1kHz sampling frequency</li>
      </ul>
    </td>
  </tr>
	<!-- Encoder Reading Method -->
  <tr> 
    <td style="vertical-align:top"> Encoder Reading Method</td>
    <td> 
			<ul>
				<li>RP2040 PIO via <code>embassy_rp::pio_programs::rotary_encoder::PioEncoder</code></li>
			</ul>
		</td>
  </tr>
	<!-- Flash Storage -->
	<tr> 
    <td style="vertical-align:top">Flash Storage</td>
    <td> 
			<ul>
				<li>Save firware config on the flash memory to simulate EEPROM via <code>sequential_storage</code></li>
			</ul>
		</td>
  </tr>
	<!-- Multicore -->
	<tr> 
    <td style="vertical-align:top">Multicore</td>
    <td> 
			<ul>
				<li>CORE 0 &rarr; USB Communication + Logger + Flash Storage</li>
				<li>CORE 1 &rarr; Motor Control (100 Hz sampling rate)</li>
			</ul>
		</td>
  </tr>

</table>
</div>

## Hardware
### Specification

<div align="center">
<table>
  <tr> 
    <th width = "125" align="center"> Components</th>
    <th width = "200" align="center"> Specification </th>
  </tr>

  <tr> 
    <td> Microcontroller</td>
    <td> Raspberry Pi Pico RP2040 </td>
  </tr>

  <tr> 
    <td> Motor</td>
    <td> Motor DC JGA25-370 12V Gearbox (1200 RPM)</td>
  </tr>

  <tr> 
    <td> Motor Driver</td>
    <td> BTS7960 </td>
  </tr>

  <tr> 
    <td > Logic Shifter</td>
    <td > Raspberry Pi Pico RP2040 </td>
  </tr>

</table>
</div>

<p align="center">
    <br>
    <img src="assets/motor_setup.jpg" width="500">
</p>

### GPIO Map


## Project Structure


## Getting Started



