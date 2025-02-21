# Simple 2 D.O.F Camera Monitor
Very simple 2 D.O.F camera monitor robot with two servo motors. 
This is my first project to use Rust + embassy-rs for microcontroller.

## Project Structure
```bash
.
├── Cargo.lock
├── Cargo.toml
├── README.md
├── build.rs
├── memory.x
└── `src`
    ├── main.rs
    ├── `resources`
    │   ├── gpio_list.rs
    │   └── mod.rs
    └── `tasks`
        ├── button.rs
        ├── control_task.rs
        ├── display.rs
        └── mod.rs

4 directories, 12 files
```

## Hardware Components
This is the main component of this project:

|Component               | Description |
|------------------------|-------------|
|Raspberry Pi Pico RP2040| I use this because embassy-rs have many examples for Pico and also Pico W |
|JIYUE Baby Monitor|Existing baby monitor camera. This is for quick development, maybe I will develop embedded camera later|
|MG996R + Bracket|Servo Motor for yaw and pitch camera movement. Controlled by varying the PWM duty cycles via PIO. I have developed my own library for this project on [rp2040-servo-pio](https://github.com/tutla53/embassy-rp-library) |
|HC-05 Bluetooth         |Bluetooth module to drive the servo motor. The remote control is using my existing remote control by using STM32F1 Bluepill and FreeRTOS on [remote-control-stm32](https://github.com/tutla53/remote-control-stm32) |
|IC2262/2272 RC Module   |RC transmitter and receiver module with 433 MHz variant. The output of this module is 5V, so we need the logic shifter to connect to Pico|
|CD4050BE or BSS138|Logic level converter to drive the PWM from 3.3V to 5V and convert RC Module output from 5V to 3.3V|

## Resources Map
Detailed resources list can be found in the `resources/gpio_list.rs`

### GPIO Map

|GPIO| Description|
|---|---|
|GP5|UART1 RX Pin connected to TX pin of HC-05 Bluetooth Module|
|GP10|PWM Output to Drive Body Servo Motor|
|GP12|PWM Output to Drive Head Servo Motor|
|GP18|Left Signal Input from RC Module (Yaw move CCW)|
|GP19|Right Signal Input from RC Module (Yaw move CW)|
|GP20|Up Signal Input from RC Module (Pitch move CCW)|
|GP21|Down Signal Input from RC Module (Pitch move CW)|

### Pheriperal Map

|Pheriperal| Description|
|---|---|
|PIO0|Driving Head Servo Motor|
|PIO1|Driving Head Servo Motor|
|UART1|Handle the HC-05 UART Communication|
|DMA_CH1|Handle the HC-05 UART Communication|

## Circuit Diagram

## Task Schematic




