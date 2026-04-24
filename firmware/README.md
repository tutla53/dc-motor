# Build the RP2040 Firmware
This project use `debian` to build the `uf2` file. This is also works on the `WSL2/debian` if you're using Windows

### Software Dependencies
- [Rust](https://www.rust-lang.org/tools/install)🦀
- [elf2uf2-rs](https://crates.io/crates/elf2uf2-rs/versions) &rarr; Converting the `elf` file to `uf2` file.
- [flip-link](https://github.com/knurling-rs/flip-link) &rarr; Add zero-cost stack overflow protection to your embedded programs
***

## Getting Started:
### Software Installation
- #### Installing `Rust`
Based on the official rust website, we can install the rust with this command:
  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  ```
- #### Installing the `elf2uf2-rs`
To install this, we need to install the `libudev` first:
  ```bash
  sudo apt install -y pkg-config libusb-1.0-0-dev libftdi1-dev && sudo apt-get install libudev-dev
  ```
Then we can install the `elf2uf2-rs` by using this command:
  ```bash
  cargo install elf2uf2-rs
  ```

- #### Installing the `flip-link`
`flip-link` is available on [crates.io](crates.io). To install it, run:
  ```bash
  cargo install flip-link
  ```

### Cloning the Repository
Clone this repository with this command:
  ```bash
  git clone https://github.com/tutla53/dc-motor.git 
  ```

### Build the Program
#### Project Structure
The structure of the project is shown on the listing below. Basically we have two different type of package:
- main &rarr; dc motor code
- playground &rarr; experimental package to test the new feature before implemented to the main. 

``` bash
.
├── .cargo
│   └── config.toml
├── Cargo.toml              # Master Cargo for all packages
├── main
│   ├── build.rs            # Build Script
│   ├── Cargo.toml          # Package Cargo for main
│   ├── memory.x            # RP2040 memory layout
│   └── src
│       ├── control
│       ├── resources
│       ├── tasks
│       └── main.rs         # DC motor main code
├── playground
│   ├── flash_storage       # flash_storage package
│   └── usb_communication   # usb_communication package
├── rustfmt.toml            # rustfmt config
├── rust-toolchain.toml     # toolchain config
└── taplo.toml              # taplo config
```
#### Build the `uf2` file
General command to build the `uf2` is by running this command:
  ```bash
  cargo run --release --package {pakage_name}
  ```
To run the main code, we can run:
  ```bash
  cargo run --release --package main
  ```
And for the playground code like usb_communication, we can run:
  ```bash
  cargo run --release --package usb_communication
  ```


<!-- NOTES -->
***
## Using `probe-rs` 
If you want to use the `probe-rs` instead of the `elf2uf2-rs` we can install it first by using this command:
#### Installing probe-rs Package
  ```bash
  curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
  ```

| :warning: WARNING          |
|:---------------------------|
|Make sure that you didn’t accidentally run `cargo add probe-rs` (which adds it as a dependency) instead of correctly installing probe-rs|

#### Edit `.cargo/config.toml`
Please select one of the runner on the `.cargo/config.toml`
```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "probe-rs run --chip RP2040"  # Directly Flash with Debug Probe
```