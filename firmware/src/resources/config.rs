/*
*  Default Firmware Config
*
*  DC Motor Properties
*  - DC Motor Gearbox Ratio 1:4.4
*  - Encoder PPR = 11
*  - Overall PPR = 48.4 PPR or 484 Pulse per 10 Rotation
*/

use super::*;

pub const N_MOTOR: usize = 2;

/* --------------------------- Motor PID Config-------------------------- */
pub const DEFAULT_PID_POS_CONFIG: PIDConfig = PIDConfig {
    kp: 25.0,
    ki: 0.0,
    kd: 5.0,
    i_limit: 1500.0,
};

pub const DEFAULT_PID_SPEED_CONFIG: PIDConfig = PIDConfig {
    kp: 2.0,
    ki: 0.8,
    kd: 10.0,
    i_limit: MOTOR_MAX_PWM_TICKS as f32,
};

/* --------------------------- Motor Properties -------------------------- */
pub const MOTOR_MAX_SPEED_CPS: i32 = 968;           // Physical Limit 1200 RPM
pub const MOTOR_CONTROL_MAX_SPEED_CPS: i32 = 1130;  // 1400 RPM for control limit

/* --------------------------- Clock and PWM Config -------------------------- */
pub const SYSTEM_FREQ_HZ: u32 = 133_000_000;                                    // 133 MHz
pub const PWM_FREQ_HZ: u32 = 25_000;                                            // 25kHz
pub const PWM_PERIOD_TICKS: u16 = (SYSTEM_FREQ_HZ / PWM_FREQ_HZ) as u16 - 1;    // 25kHz Period = (125_000_000 (Pico clock)/25_000(Frequency)) -1
pub const MOTOR_MAX_PWM_TICKS: i32 = PWM_PERIOD_TICKS as i32;                   // Full Range

/* --------------------------- Control Config -------------------------- */
pub const TIME_SAMPLING_US: u64 = 5000;                                     // Control Loop Frequency
pub const TICKS_TO_CPS: f32 = 1_000_000.0_f32 / TIME_SAMPLING_US as f32;    // Calculating Speed from Ticks
pub const SPEED_FILTER_WINDOW: usize = 1 << 3;                              // Must be 2^n

/* --------------------------- Motor Steady-State Criteria -------------------------- */
pub const POS_TOLERANCE_COUNT: i32 = 5;
pub const SPEED_TOLERANCE_CPS: i32 = 2;
pub const SETTLE_TICKS: u32 = 20;

/* --------------------------- USB Communication-------------------------- */
pub const USB_BUFFER_SIZE:  usize = 64;
pub const EVENT_CHANNEL_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;
pub const LOG_BUFFER_SIZE: usize = 256;

/* --------------------------- Communication Channels-------------------------- */
pub static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, USB_BUFFER_SIZE> = Channel::new();
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, EventList, EVENT_CHANNEL_SIZE> = Channel::new();
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> = Channel::new();

/* --------------------------- USB Builder-------------------------- */
pub static USB_STATE: StaticCell<State> = StaticCell::new();
pub static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
pub static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
pub static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/* --------------------------- Multicore Executor -------------------------- */
pub static mut CORE1_STACK: Stack<4096> = Stack::new();
pub static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
pub static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

/* --------------------------- FLASH STORAGE CONFIG -------------------------- */
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;  // 2MB
pub const SECTOR_SIZE: u32 = 4096;              // 4KB
pub const STORAGE_START: u32 = FLASH_SIZE as u32 - (4 * SECTOR_SIZE); 
pub const STORAGE_END: u32 = FLASH_SIZE as u32;