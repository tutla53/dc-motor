# DC Motor Project

<div align="center">
  <a href="../../firmware/README.md"><img src="../../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../../firmware/README.md"><img src="../../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
</div>
<div align="center">
  Firmware Documentation
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="800" height="1">
</div>

## Overview

The main firmware controls two DC motor channels on the RP2040. We use
`embassy-rs` to run USB communication and logging on Core 0, while Core 1 runs
the motor-control and encoder tasks. Each motor has its own command queue,
feedback, PID settings, and control state.

This page explains how the firmware modules work together. For toolchain setup,
building, flashing, and creating a playground package, see the
[Firmware Guide](../../firmware/README.md).

## Tasks

The task assignments below follow
[main.rs](../../firmware/main/src/main.rs). Both cores use thread-mode
executors. `EXECUTOR_HIGH` and its interrupt handler are declared, but the
current initialization does not start that executor or assign tasks to it.

<div align="center">
  <table align="center">
    <tr><th align="center">Core</th><th align="center">Task</th><th align="center">Responsibility</th></tr>
    <tr><td align="center">0</td><td align="left"><code>usb_device_task</code></td><td align="left">Run the USB device.</td></tr>
    <tr><td align="center">0</td><td align="left"><code>usb_rx_task</code></td><td align="left">Receive USB data and pass packets to the command task.</td></tr>
    <tr><td align="center">0</td><td align="left"><code>usb_command_task</code></td><td align="left">Parse commands, update motor/configuration state, and prepare responses.</td></tr>
    <tr><td align="center">0</td><td align="left"><code>usb_traffic_controller_task</code></td><td align="left">Select responses, events, and logger records for transmission.</td></tr>
    <tr><td align="center">0</td><td align="left"><code>usb_tx_task</code></td><td align="left">Send prepared packets through USB.</td></tr>
    <tr><td align="center">0</td><td align="left"><code>firmware_logger_task</code></td><td align="left">Sample telemetry for the selected motor.</td></tr>
    <tr><td align="center">0</td><td align="left"><code>heartbeat_task</code></td><td align="left">Toggle the onboard LED; this is not a communication watchdog.</td></tr>
    <tr><td align="center">1</td><td align="left"><code>motor0_task</code>, <code>motor1_task</code></td><td align="left">Run each motor's periodic control loop.</td></tr>
    <tr><td align="center">1</td><td align="left"><code>encoder0_task</code>, <code>encoder1_task</code></td><td align="left">Read PIO encoder direction events and update position.</td></tr>
  </table>
</div>

### Initialization

`main.rs` configures the system clock and GPIO resources, creates the flash
storage and USB interface, then builds both motors and encoders. Motor
construction loads stored settings before the control tasks start. The encoders
use PIO0 state machines 0 and 1. Core 1 is then started, followed by the Core 0
communication, logger, and heartbeat tasks. Motors initialize disabled.

## Resources

### GPIO List

[resources/gpio_list.rs](../../firmware/main/src/resources/gpio_list.rs)
uses `assign_resources!` to group pins and PWM slices by motor. These are RP2040
GPIO numbers, not connector pin numbers.

<div align="center">
  <table align="center">
    <tr><th align="center">Resource</th><th align="center">Motor 0</th><th align="center">Motor 1</th></tr>
    <tr><td align="center">PWM CW</td><td align="center">GPIO 15</td><td align="center">GPIO 3</td></tr>
    <tr><td align="center">PWM CCW</td><td align="center">GPIO 14</td><td align="center">GPIO 2</td></tr>
    <tr><td align="center">Encoder A</td><td align="center">GPIO 6</td><td align="center">GPIO 4</td></tr>
    <tr><td align="center">Encoder B</td><td align="center">GPIO 7</td><td align="center">GPIO 5</td></tr>
    <tr><td align="center">PWM slice</td><td align="center">7</td><td align="center">1</td></tr>
    <tr><td align="center">PIO0 state machine</td><td align="center">0</td><td align="center">1</td></tr>
  </table>
</div>

The Motor 1 pin group is still marked `Still Dummy` in the source; verify its
wiring before using it. GPIO 25 drives the onboard LED. DMA channel 0 is used
for flash operations. Interrupt bindings for USB, PIO, and DMA are also defined
in `gpio_list.rs`.

### Firmware Config

[resources/config.rs](../../firmware/main/src/resources/config.rs) defines
the firmware timing, limits, default PID parameters, buffers, and shared
resources.

<div align="center">
  <table align="center">
    <tr><th align="left">Setting</th><th align="left">Current value</th><th align="left">Meaning</th></tr>
    <tr><td align="left"><code>N_MOTOR</code></td><td align="left">2</td><td align="left">Motor channels, IDs 0 and 1.</td></tr>
    <tr><td align="left"><code>SYSTEM_FREQ_HZ</code></td><td align="left">133 MHz</td><td align="left">Configured system clock.</td></tr>
    <tr><td align="left"><code>PWM_FREQ_HZ</code></td><td align="left">25 kHz</td><td align="left">Configured H-bridge PWM frequency.</td></tr>
    <tr><td align="left"><code>PWM_PERIOD_TICKS</code></td><td align="left">5319</td><td align="left">PWM top value derived from the clock and PWM frequency.</td></tr>
    <tr><td align="left"><code>TIME_SAMPLING_US</code></td><td align="left">1000 &#181;s</td><td align="left">Nominal 1 kHz motor-control loop.</td></tr>
    <tr><td align="left"><code>SPEED_FILTER_WINDOW</code></td><td align="left">32 samples</td><td align="left">Speed-estimation window, nominally 32 ms.</td></tr>
    <tr><td align="left"><code>DEFAULT_MOTOR_CONTROL_MAX_SPEED_PPS</code></td><td align="left">807 pulses/s</td><td align="left">Default controller speed limit.</td></tr>
    <tr><td align="left"><code>PHYSICAL_MOTOR_MAX_SPEED_PPS</code></td><td align="left">1129 pulses/s</td><td align="left">Software upper bound for accepted maximum-speed settings.</td></tr>
    <tr><td align="left"><code>POS_TOLERANCE_PULSE</code></td><td align="left">5 pulses</td><td align="left">Position tolerance used for move completion.</td></tr>
    <tr><td align="left"><code>SPEED_TOLERANCE_PPS</code></td><td align="left">2 pulses/s</td><td align="left">Speed tolerance used for move completion.</td></tr>
    <tr><td align="left"><code>SETTLE_TICKS</code></td><td align="left">50 ticks</td><td align="left">Required settling duration, nominally 50 ms.</td></tr>
  </table>
</div>

<div align="center">
  <table align="center">
    <tr><th align="center">Default PID</th><th align="center">kp</th><th align="center">ki</th><th align="center">kd</th><th align="center">i_limit</th></tr>
    <tr><td align="center">Position</td><td align="center">25.0</td><td align="center">0.0</td><td align="center">5.0</td><td align="center">1500.0</td></tr>
    <tr><td align="center">Speed</td><td align="center">2.0</td><td align="center">0.16</td><td align="center">25.0</td><td align="center">26595.0</td></tr>
  </table>
</div>

These are configured values, not measured timing or mechanical limits. Stored
PID and maximum-speed settings can override the compiled defaults. The
firmware version is derived from `main/Cargo.toml`.

The shared [config/motor_config.toml](../../config/motor_config.toml) belongs
to the Rust host and Python identification tool. Firmware does not read that
file. Keep host conversions and model settings consistent with the hardware,
but change firmware defaults in `resources/config.rs`.

### Inter-Task and Inter-Core Communication

The choice of synchronization depends on what is shared: a latest value, an
ordered command, or a configuration structure.

<div align="center">
  <table align="center">
    <tr><th align="left">Mechanism</th><th align="left">Used for</th><th align="left">Behavior in this firmware</th></tr>
    <tr><td align="left"><code>core::sync::atomic</code></td><td align="left">Position, speed, commanded values, move-done state, maximum speed, and logger state.</td><td align="left">Individual scalar loads/stores. Telemetry fields are read separately, so a full record is not an atomic snapshot.</td></tr>
    <tr><td align="left"><code>portable_atomic::AtomicBool</code></td><td align="left">Motor enable/disable requests and configuration-update flags.</td><td align="left">Request consumption uses compare-exchange; update flags use swap. These operations use the configured critical-section support.</td></tr>
    <tr><td align="left"><code>Channel&lt;CriticalSectionRawMutex, ...&gt;</code></td><td align="left">Motor commands, USB packets, responses, events, and log records.</td><td align="left">Bounded queues. Async <code>send</code> waits for capacity; <code>try_send</code> returns immediately if full.</td></tr>
    <tr><td align="left"><code>Mutex&lt;CriticalSectionRawMutex, PIDConfig&gt;</code></td><td align="left">Each motor&#39;s position and speed PID settings.</td><td align="left">Protects the complete configuration during read or update.</td></tr>
    <tr><td align="left"><code>Mutex&lt;ThreadModeRawMutex, ...&gt;</code></td><td align="left">Flash storage on Core 0.</td><td align="left">Serializes access to the storage instance.</td></tr>
    <tr><td align="left"><code>StaticCell&lt;T&gt;</code></td><td align="left">Executor storage, USB descriptors, and USB class state.</td><td align="left">Provides static storage initialized at runtime; it is not a message queue.</td></tr>
  </table>
</div>

For example, the command task updates a PID structure under its mutex and sets
an update flag. The motor task consumes the flag and applies the new settings
at a control-loop boundary. Normal movement commands instead enter that
motor's 16-entry command queue.

See [motor_handler.rs](../../firmware/main/src/motor/motor_handler.rs) for the
motor state and [logger_handler.rs](../../firmware/main/src/firmware_logger/logger_handler.rs)
for the logger state. The motor ID is an ordinary `u8` in `MotorHandler`; the
logger's selected motor ID uses `AtomicU8`.

## Communication

The firmware uses USB CDC ACM with binary packets. Its USB identity is configured
in `main.rs`: VID `0xC0DE`, PID `0xCAFE`, and serial number `12345678`.
The host uses that serial number prefix for discovery.

### Command Processing

1. `usb_rx_task` receives a packet and sends it to `USB_RX_CHANNEL`.
2. `usb_command_task` calls `CommandHandler` to decode and validate it.
3. The handler reads state, updates configuration, or queues a motor command.
4. The response is placed in `CMD_CHANNEL`.
5. `usb_traffic_controller_task` selects a response, event, or logger record
   and passes the encoded packet to `USB_TX_CHANNEL`.
6. `usb_tx_task` transmits the packet to the host.

`CMD_CHANNEL` carries responses despite its name. The traffic controller uses
`select3` across the three sources; it does not reserve a fixed bandwidth for
telemetry.

### Packet Format

Multibyte numeric values use little-endian encoding. The configured USB packet
buffer is 64 bytes.

<div align="center">
  <table align="center">
    <tr><th align="left">Packet</th><th align="left">Format</th><th align="left">Meaning</th></tr>
    <tr><td align="left">Command</td><td align="left"><code>0xFF, opcode:u8, arguments...</code></td><td align="left">Request from the host.</td></tr>
    <tr><td align="left">Response</td><td align="left"><code>0xFF, error:u8, opcode:u8, payload...</code></td><td align="left">Command result; error 0 means success.</td></tr>
    <tr><td align="left">Event</td><td align="left"><code>0xFE, event_code:u8, motor_id:u8</code></td><td align="left">Asynchronous event; event 0 is <code>MOVE_MOTOR_DONE</code>.</td></tr>
    <tr><td align="left">Logger</td><td align="left"><code>0xFD, sequence:u8, timestamp_ms:u32, values:[i32;5]</code></td><td align="left">One 26-byte telemetry record.</td></tr>
  </table>
</div>

For example, `get_motor_pos(0)` has opcode 11. A successful response reporting
100 pulses is:

```text
Host request:       FF 0B 00
Firmware response:  FF 00 0B 64 00 00 00
```

Command names and argument definitions are in
[DCMotor.toml](../../DeviceOpFuncs/DCMotor.toml). Firmware maintains its
[API definitions](../../firmware/main/src/communication/api_config.rs) and
[command handler](../../firmware/main/src/communication/command_handler.rs)
separately. Update both sides when changing a command or protocol version.

### Command Results

Movement commands are rejected while the motor is disabled or when its normal
command queue is full. The parser checks argument types and lengths, including
unexpected trailing bytes. PID settings and maximum speed are validated before
being accepted. Trapezoidal requests reject zero velocity or acceleration.

A successful movement response means the request was accepted; it does not
mean the move has finished. Position completion is reported by an event.
Enable and disable are requests processed by the motor task. Disable uses a
separate flag, taking precedence over the normal command queue.

## Firmware Logger

There is one shared `LOGGER`, selecting one motor at a time. The
`start_logger(motor_id, time_sampling)` API takes an interval in milliseconds,
with a minimum of 1 ms. The logger interval is separate from the 1 ms control
period; changing it does not change the controller frequency.

On detecting a new logging session,
[firmware_logger_task](../../firmware/main/src/tasks/logger.rs) clears its
queue, resets the sequence number and elapsed-time origin, and samples the
selected motor's shared values. While inactive, it sleeps in 100 ms intervals,
so sampling does not necessarily start at the instant the command is acknowledged.

The five values are always packed in this order:

<div align="center">
  <table align="center">
    <tr><th align="center">Index</th><th align="left">Value</th><th align="left">Firmware unit</th></tr>
    <tr><td align="center">0</td><td align="left">Current position</td><td align="left">Encoder pulses</td></tr>
    <tr><td align="center">1</td><td align="left">Current speed</td><td align="left">Pulses/s</td></tr>
    <tr><td align="center">2</td><td align="left">Commanded position</td><td align="left">Encoder pulses</td></tr>
    <tr><td align="center">3</td><td align="left">Commanded speed</td><td align="left">Pulses/s</td></tr>
    <tr><td align="center">4</td><td align="left">Commanded PWM</td><td align="left">Signed PWM ticks</td></tr>
  </table>
</div>

The timestamp is milliseconds since the logger session began. The 8-bit
sequence number wraps after 255. Records enter a 2048-entry queue using
`try_send`; a full queue drops the new record. Stopping logging does not
guarantee that every queued record is transmitted.

The firmware sends binary telemetry. The host selects exported columns through
`LogMask`, converts units using `DCMotor.toml`, and writes CSV and plots. See
[Logs and Model Comparison](../../rust_script/README.md#logs-and-model-comparison).

## Flash Storage

[storage_handler.rs](../../firmware/main/src/flash_storage/storage_handler.rs)
uses `postcard` to serialize settings and `sequential_storage` to store them as
key/value records. Flash access uses DMA channel 0 and a shared mutex.

### Memory Layout

[main/memory.x](../../firmware/main/memory.x) reserves the last 16 KiB of the
configured 2 MiB flash for settings. Ranges below use an exclusive end address.

<div align="center">
  <table align="center">
    <tr><th align="left">Region</th><th align="left">Memory-mapped address range</th><th align="left">Size</th></tr>
    <tr><td align="left"><code>BOOT2</code></td><td align="left"><code>0x10000000</code> to <code>0x10000100</code></td><td align="left">256 bytes</td></tr>
    <tr><td align="left">Application <code>FLASH</code></td><td align="left"><code>0x10000100</code> to <code>0x101FC000</code></td><td align="left">2 MiB minus 16 KiB minus 256 bytes</td></tr>
    <tr><td align="left"><code>STORAGE</code></td><td align="left"><code>0x101FC000</code> to <code>0x10200000</code></td><td align="left">16 KiB</td></tr>
    <tr><td align="left"><code>RAM</code></td><td align="left"><code>0x20000000</code> to <code>0x20042000</code></td><td align="left">264 KiB</td></tr>
  </table>
</div>

The flash driver uses offsets rather than memory-mapped addresses:
`STORAGE_START = 0x001FC000` and `STORAGE_END = 0x00200000`.
Keep `memory.x` and the storage constants in `resources/config.rs` consistent
when changing the layout.

### Stored Settings

Each motor has three records. The key is `(motor_id << 4) | config_type`:

<div align="center">
  <table align="center">
    <tr><th align="left">Setting</th><th align="center">Type ID</th><th align="center">Motor 0 key</th><th align="center">Motor 1 key</th></tr>
    <tr><td align="left">Speed PID</td><td align="center">0</td><td align="center"><code>0x00</code></td><td align="center"><code>0x10</code></td></tr>
    <tr><td align="left">Position PID</td><td align="center">1</td><td align="center"><code>0x01</code></td><td align="center"><code>0x11</code></td></tr>
    <tr><td align="left">Maximum speed</td><td align="center">2</td><td align="center"><code>0x02</code></td><td align="center"><code>0x12</code></td></tr>
  </table>
</div>

Maximum speed is stored as the legacy signed `i32` type, then checked before
conversion to the runtime `u32` value. PID values are checked before application
to the controllers. If a record cannot be fetched or decoded, `load_config`
returns its default and attempts to save that default to flash.

Normal PID and maximum-speed setters update runtime settings. The
`save_configuration` command persists the current settings for both motors.
`reset_configuration` applies and saves the compiled defaults for both motors.
Neither command takes a motor ID.

These writes are separate operations. A failed write returns
`FlashStorageError`, but records already written are not rolled back. Boot-time
default writes can also fail without preventing the default from being used.

## Motor Control

The control loop is implemented in
[tasks/dc_motor.rs](../../firmware/main/src/tasks/dc_motor.rs), using PID,
filtering, and motion-profile code from
[crates/motor-control](../../crates/motor-control/).

At each nominal 1 ms tick, `run_motor_task` reads position, estimates speed,
handles disable/enable and pending configuration changes, then consumes at most
one normal command when enabled. Feedback continues to update while disabled.

<div align="center">
  <table align="center">
    <tr><th align="left">Mode</th><th align="left">Control behavior</th></tr>
    <tr><td align="left">Open loop</td><td align="left">Apply signed PWM, clamped to the configured PWM limit.</td></tr>
    <tr><td align="left">Speed</td><td align="left">Speed PID converts the speed error into PWM.</td></tr>
    <tr><td align="left">Position, step</td><td align="left">Position PID produces the speed target; speed PID produces PWM.</td></tr>
    <tr><td align="left">Position, trapezoidal</td><td align="left">A profile supplies position targets to the same cascaded controllers.</td></tr>
    <tr><td align="left">Stop</td><td align="left">Set PWM to zero.</td></tr>
  </table>
</div>

The encoder task adds or subtracts one pulse per PIO direction event. The
control task estimates speed from a 32-sample moving-average filter. Position
control uses `I32F32`; speed control uses `I16F16` fixed-point arithmetic.

Disable drives PWM to zero, resets controllers and motion state, and drains
queued commands. A normal Stop command goes through the motor queue. Neither
USB disconnection nor the heartbeat task automatically requests disable.

For position moves, the completion check requires position and speed to remain
within their configured tolerances for 50 ticks. Sending `MOVE_MOTOR_DONE` does
not itself exit position control. Also, profile-construction failure currently
uses the same event, and event enqueue uses `try_send`; the host must retain a
timeout rather than treating event delivery as guaranteed success.

See [Control Design](../02-Control-Design.md) for controller theory.
The descriptions here follow source code; they do not establish measured
timing or hardware performance.

#

<div align="center">
  <a href="../../firmware/README.md"><img src="../../assets/logo/left-chevron.png" alt="<< Prev" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
  <a href="../../firmware/README.md"><img src="../../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="450" height="1">
</div>
<div align="center">
  Firmware Documentation
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"" width="800" height="1">
</div>
