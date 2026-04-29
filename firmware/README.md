# Build the RP2040 Firmware
This project use `debian` to build the `uf2` file. This is also works on the `WSL2/debian` if you're using Windows

### Software Dependencies
- [Rust](https://www.rust-lang.org/tools/install)🦀
- [probe-rs](https://probe.rs/) &rarr; Embedded debugging toolkit
- [elf2uf2-rs](https://crates.io/crates/elf2uf2-rs/versions) &rarr; Converting the `elf` file to `uf2` file.
- [flip-link](https://github.com/knurling-rs/flip-link) &rarr; Add zero-cost stack overflow protection to your embedded programs
***

## Getting Started:
### Software Installation

- #### Installing `Rust`
  - Based on the official rust website, we can install the rust with this command:
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```

- #### Installing `probe-rs`
  - :warning: Make sure that you didn’t accidentally run `cargo add probe-rs` (which adds it as a dependency) instead of correctly installing probe-rs
    ```bash
    curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
    ```

- #### Installing the `elf2uf2-rs`
  - To install this, we need to install the `libudev` first:
    ```bash
    sudo apt install -y pkg-config libusb-1.0-0-dev libftdi1-dev && sudo apt-get install libudev-dev
    ```
  - Then we can install the `elf2uf2-rs` by using this command:
    ```bash
    cargo install elf2uf2-rs
    ```

- #### Installing the `flip-link`
  - `flip-link` is available on [crates.io](crates.io). To install it, run:
    ```bash
    cargo install flip-link
    ```

### Cloning the Repository
- Clone this repository with this command:
  ```bash
  git clone https://github.com/tutla53/dc-motor.git 
  ```

### Build the Program
#### Project Structure
- The structure of the project is shown on the listing below. Basically we have two different type of package:
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
- To build the project you can select from the options below:
<table>
  <tr>
			<th width = "300" align="center"> Notes</th>
			<th width = "520" align="center"> probe-rs</th>
      <th width = "520" align="center"> elf2uf2-rs</th>
  </tr>
  <tr>
    <td> General commmand structure</td>
    <td><code>cargo run --release --package {pakage_name}</code></td>
    <td><code>cargo run-uf2 {pakage_name}</code></td>
  </tr>
  <tr>
    <td> Run <code>main</code></td>
    <td><code>cargo run --release --package main</code></td>
    <td><code>cargo run-uf2 main</code></td>
  </tr>
  <tr>
    <td> Run Playground</td>
    <td><code>cargo run --release --package usb_communication</code></td>
    <td><code>cargo run-uf2 usb_communication</code></td>
  </tr>
</table>
