# DC Motor Project

<div align="center">
  <a href="../../firmware/README.md"><img src="../../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../../firmware/README.md"><img src="../../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="02-System-Identification.md"><img src="../../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  Firmware Documentation
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  System Identification
</div>
    
#

## Tasks
<div align="center">
	<table>
		<tr> 
			<th width=100 align="center"> Parameter</th>
			<th width=600 align="center"> Value </th>
		</tr>
		<tr> 
      <td align="left"> Task List</td>
      <td align="left">
        CORE 0
        <ul>
          <li>
            USB Communication
            <ul>
              <li><code>usb_device_task</code> : Start USB Communication</li>
              <li><code>usb_rx_task</code> : Handling Received Message</li>
              <li><code>usb_command_task</code> : Processing and Executing the Message</li>
              <li><code>usb_traffic_controller_task</code> : Controlling the Firmware Response Order</li>
              <li><code>usb_tx_task</code> : Handling Firmware Response</li>
            </ul>
          </li>
          <li>
            Firmware Logger
            <ul>
              <li><code>firmware_logger_task</code> : Handling Firmware Data Telemetry</li>
            </ul>
          </li>
          <li>
            Heartbeat
            <ul>
              <li><code>heartbeat_task</code> : LED Indicator</li>
            </ul>
          </li>          
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
    <tr> 
	    <td align="left">Flash Storage</td>
	    <td align="left"><ul><li>Save firmware config on the flash memory to simulate EEPROM via <code>sequential_storage</code></li></ul></td>
	  </tr> 
	</table>
</div>

  <!-- ``` bash
  .
  ├── .cargo
  │   └── config.toml
  ├── Cargo.toml              # Master Cargo for all packages
  ├── main
  │   ├── build.rs            # Build Script
  │   ├── Cargo.toml          # Package Cargo for main
  │   ├── memory.x            # RP2040 memory layout
  │   └── src
  │       ├── communication
  │       ├── firmware_logger
  │       ├── flash_storage
  │       ├── motor
  │       ├── resources
  │       ├── tasks
  │       └── main.rs         # DC motor main code
  └── playground
      ├── flash_storage       # flash_storage package
      └── usb_communication   # usb_communication package
  ``` -->

## Resources
### GPIO LIST
`assign_resources!`
### Firmware Config
`config.rs`
### Inter-Task and Inter-Core Communication Method
<div align="center">
	<table>
		<tr> 
			<th width=200 align="center"> Crates </th>
      <th width=300 align="center"> Description </th>
      <th width=500 align="center"> Variables </th>
		</tr>
		<tr> 
      <td align="left">
        <a href="https://rust.docs.kernel.org/6.1/core/sync/atomic/index.html"> <code>core::sync::atomic</code></a>
      </td>
      <td>Primitive shared-memory communication between threads, and are the building blocks of other concurrent types</td>
      <td align="left">
        <code>AtomicI32</code>
        <ul>
          <li>Current Position Data</li>
          <li>Current Speed Data</li>
          <li>Current Commanded Position Data</li>
          <li>Current Commanded Speed Data</li>
          <li>Current Commanded PWM Data</li>
        </ul>
        <code>AtomicU8</code>
        <ul>
          <li>Motor ID</li>
        </ul>
        <code>AtomicBool</code>
        <ul>
          <li>Motor Move Done Status</li>
          <li>Logger Active Status</li>
        </ul>        
      </td>
    </tr>
		<tr> 
      <td align="left">
        <a href="https://docs.rs/portable-atomic/1.15.0/portable_atomic/"> <code>portable_atomic::AtomicBool</code></a>
      </td>
      <td align="left">Similar with <code>core::sync::atomic</code> but specifically for request and update flags</td>
      <td align="left">
        <ul>
          <li>Motor Enabled Status</li>
          <li>Motor Enable/Disable Request</li>
          <li>Position PID Flag</li>
          <li>Speed PID Flag</li>
          <li>Maximum Speed Update Flag</li>
        </ul>
      </td>
    </tr>
		<tr> 
      <td align="left">
        <a href="https://docs.embassy.dev/embassy-sync/git/default/index.html"><code>embassy_sync</code></a>
      </td>
      <td>Synchronization primitives and data structures with async support</td>
      <td align="left">     
        <a href="https://docs.embassy.dev/embassy-sync/git/default/channel/index.html"><code>embassy_sync::channel</code></a>
        : Transfer time-sensitve data (critical)
        <ul>
          <li>Event Data</li>
          <li>Command Data</li>
          <li>Firmware Logger Data</li>
          <li>Motor Command Data</li>          
        </ul>
        </ul>
        <a href="https://docs.embassy.dev/embassy-sync/git/default/mutex/index.html"><code>embassy_sync::mutex</code></a>
        : Transfer non time-sensitve data
        <ul>
          <li>Position Control PID Config</li>
          <li>Speed Control PID Config</li>
        </ul>         
      </td>
    </tr>   
    <tr>
      <td align="left">
        <a href="https://docs.rs/static_cell/2.1.1/static_cell/"> <code>static_cell::StaticCell<T></code></a>
      </td>
      <td align="left">Statically allocated, initialized at runtime cell</td>
      <td align="left"></td>    
    </tr>                                  
	</table>
</div>

## Communication

## Firmware Logger

## Flash Storage
### Memory Layout

## Motor Control
TODO: REFER TO "../docs/01-Control-Implementation.md"

#
<div align="center">
  <a href="README.md"><img src="../../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../README.md"><img src="../../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="02-System-Identification.md"><img src="../../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  DC Motor System
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="730" height="1">
  System Identification
</div>
    
#
