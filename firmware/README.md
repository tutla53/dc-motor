# RP2040 Firmware Documentation

<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="../docs/firmware/01-dc-motor-project.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="700" height="1">
  DC Motor Project
</div>
	
#

## Project Overview

This workspace contains the RP2040 firmware built with `embassy-rs`. The main
application handles motor control, encoder feedback, USB communication, firmware
logging, and configuration storage. We use separate playground packages to try
new features before integrating them into the motor firmware.

The desktop application is documented in [Rust Script](../rust_script/README.md).
It sends commands through the API defined in
[DeviceOpFuncs/DCMotor.toml](../DeviceOpFuncs/DCMotor.toml).

## Project Workflow

```text
dc-motor/
├── crates/                        # Shared libraries, outside firmware/
│   ├── motor-control/             # PID, filtering, and motion profiles
│   └── usb-comm/                  # Packet encoding and decoding
└── firmware/
    ├── .cargo/config.toml         # Build target, linker, and hardware runner
    ├── Cargo.toml                 # Workspace members and shared dependencies
    ├── rust-toolchain.toml        # Rust toolchain and RP2040 target
    ├── main/                      # Main DC motor firmware
    │   ├── build.rs               # Linker setup
    │   ├── Cargo.toml             # Package dependencies
    │   ├── memory.x               # Flash and RAM layout
    │   └── src/
    └── playground/
        ├── flash_storage/         # Currently an onboard LED blink example
        └── usb_communication/     # USB communication experiment
```

The `flash_storage` package name describes its intended topic; its current
entry point only blinks GPIO 25. The main application's flash-storage
implementation is under `main/src/flash_storage/`.

For a new feature, use this workflow:

1. Develop a focused experiment in `playground/`.
2. Check and build the selected package before testing it on hardware.
3. Move reusable, hardware-independent logic into the repository's `crates/`.
4. Integrate the feature into `main/` and check the affected packages again.

### Main Firmware Structure

| Path under `main/src/` | Purpose |
| :--- | :--- |
| [main.rs](main/src/main.rs) | Initialize peripherals, shared resources, and task executors. |
| [resources/](main/src/resources/) | GPIO assignments, timing, configuration, and communication channels. |
| [tasks/](main/src/tasks/) | Motor control, encoders, USB, logger, and heartbeat tasks. |
| [motor/](main/src/motor/) | Motor commands, state, and configuration access. |
| [communication/](main/src/communication/) | Firmware API definitions and command handling. |
| [firmware_logger/](main/src/firmware_logger/) | Motor telemetry collection. |
| [flash_storage/](main/src/flash_storage/) | Persistent configuration storage. |

Core 0 handles communication, logging, and heartbeat tasks. Core 1 runs the
motor-control and encoder tasks for two motors. The configured control period
is 1 ms, with open-loop PWM, closed-loop speed, step position, and trapezoidal
position modes. See [DC Motor Project](../docs/firmware/01-dc-motor-project.md)
for the task overview.

## Project Builder

### Prerequisites

Use Rust through `rustup`, with `flip-link` available on your PATH.
[rust-toolchain.toml](rust-toolchain.toml) selects stable Rust, the
`thumbv6m-none-eabi` target, and the development components. Hardware flashing
through the configured runner also requires `probe-rs` and a compatible SWD
debug probe connected to the RP2040.

### Check and Build

From the repository root:

```powershell
cd firmware
cargo check --locked -p main
cargo build --release --locked -p main
```

These commands compile without flashing or running the board. The release
binary is saved at `target/thumbv6m-none-eabi/release/main`.

Select a playground package by its Cargo package name:

```powershell
cargo check --locked -p flash_storage
cargo build --release --locked -p usb_communication
```

For a workspace check, use:

```powershell
cargo check --locked --workspace
```

Run these commands from `firmware/` so Cargo uses its target and linker settings.
Each package's `build.rs` makes `memory.x` available to the linker and selects
the embedded linker scripts. Keep these files when creating another package.

### Flash and Run

The configured runner is `probe-rs run --chip RP2040`. The following command
flashes and starts the main firmware on the connected board:

```powershell
cargo run --release --locked -p main
```

Use it only when ready to operate the connected hardware. For a playground,
replace `main` with its package name. Debug output uses `defmt` over RTT;
the default log level is set in [.cargo/config.toml](.cargo/config.toml).

The main firmware starts motors disabled, but starting firmware can initialize
stored configuration. Disconnecting USB does not automatically stop an active
motor. Check the GPIO assignments and the experiment's behavior before running
either the main application or a playground.

## How to Create a New Project

For a simple experiment, copy the existing blink package. From `firmware/`,
choose a new directory name that does not already exist:

```powershell
Copy-Item -LiteralPath playground/flash_storage -Destination playground/encoder_test -Recurse
```

In `playground/encoder_test/Cargo.toml`, change both the package name and binary
name, and update the description:

```toml
[package]
name = "encoder_test"
version = "0.1.0"
description = "RP2040 encoder experiment"

# Keep the inherited package settings and dependencies from the copied file.

[[bin]]
name = "encoder_test"
path = "src/main.rs"
test = false
bench = false
```

Add the new package to the existing members list in `firmware/Cargo.toml`:

```toml
[workspace]
members = [
    "main",
    "playground/flash_storage",
    "playground/usb_communication",
    "playground/encoder_test",
]
resolver = "2"
```

Keep the copied `build.rs` and `memory.x`. Replace the blink code in
`src/main.rs` with the experiment, retaining the embedded entry-point setup.
Describe the experiment's purpose and pins in a short source comment.

```powershell
cargo check -p encoder_test
cargo build --release --locked -p encoder_test
```

The first check may update `Cargo.lock` for the new package. Review that change;
subsequent checks can use `--locked`.

### Dependencies Setting — Cargo.toml

Dependencies shared by several firmware packages belong in
`[workspace.dependencies]` in `firmware/Cargo.toml`. For example, the existing
Embassy timer dependency is declared there:

```toml
[workspace.dependencies]
embassy-time = { version = "0.5.0", features = ["defmt", "defmt-timestamp-uptime"] }
```

A package enables that dependency in its own `Cargo.toml`:

```toml
[dependencies]
embassy-time.workspace = true
```

This keeps the version and features in one place. Dependencies used by only
one package can remain in that package's manifest.

The local control and communication libraries follow the same pattern. Their
workspace paths are `../crates/motor-control` and `../crates/usb-comm`, relative
to `firmware/Cargo.toml`. A consuming package uses:

```toml
[dependencies]
motor-control.workspace = true
usb-comm.workspace = true
```

## Configuration and Firmware API

Firmware timing, limits, and default PID settings are defined in
[main/src/resources/config.rs](main/src/resources/config.rs). GPIO assignments
are in [gpio_list.rs](main/src/resources/gpio_list.rs). Stored device settings
can override defaults when the firmware initializes.

The repository's [config/motor_config.toml](../config/motor_config.toml) is
shared by the desktop application and Python identification tool. Firmware
does not load it; changing that TOML does not change firmware defaults or
device flash.

When changing a USB command, keep
[DCMotor.toml](../DeviceOpFuncs/DCMotor.toml),
[api_config.rs](main/src/communication/api_config.rs), and
[command_handler.rs](main/src/communication/command_handler.rs) synchronized.
Check the opcode, argument order and types, response payload, and protocol
version. Rebuild the desktop application after changing the contract.

## Further Documentation

- [Project Architecture](../README.md)
- [DC Motor Tasks](../docs/firmware/01-dc-motor-project.md)
- [Rust Host Application](../rust_script/README.md)
- [Control Design](../docs/02-Control-Design.md)
- [Control Implementation](../docs/03-Control-Implementation.md)

#
<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="../README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="../docs/firmware/01-dc-motor-project.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="700" height="1">
  DC Motor Project
</div>
	
#
