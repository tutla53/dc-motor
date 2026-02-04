/* 
* Global Resources 
*/

// Library
use defmt_rtt as _;
use panic_probe as _;
use core::sync::atomic::AtomicI32;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/* --------------------------- USB Header -------------------------- */
pub const COMMAND_HEADER: u8 = 0xFF; 
pub const EVENT_HEADER: u8 = 0xFE;
pub const LOGGER_HEADER: u8 = 0xFD;

/* --------------------------- Variables -------------------------- */
pub const USB_BUFFER_SIZE:  usize = 64;
pub const EVENT_CHANNEL_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;
pub const LOG_BUFFER_SIZE: usize = 256;

/* --------------------------- Handlers-------------------------- */
pub static MOTOR_0: MotorHandler = MotorHandler::new(0);
pub static LOGGER: LoggerHandler = LoggerHandler::new();

/* --------------------------- Channels-------------------------- */
pub static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, USB_BUFFER_SIZE> = Channel::new();
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, EventList, EVENT_CHANNEL_SIZE> = Channel::new();
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> = Channel::new();

/* --------------------------- ENUM -------------------------- */
#[derive(Clone, Copy, PartialEq)]
pub enum Shape {
    Step(i32),
    Trapezoidal(f32, f32, f32),
}

#[derive(Clone, Copy, PartialEq)]
pub enum MotorCommand {
    SpeedControl(i32),
    PositionControl(Shape),
    OpenLoop(i32),
    Stop,
}

#[derive(Clone, Copy)]
pub enum EventList {
    MotorMoveDone(u8),
}

/* --------------------------- Struct -------------------------- */
#[derive(Clone, Copy)]
pub struct PosPIDConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub kp_speed: f32,
    pub ki_speed: f32,
    pub kd_speed: f32,
}

#[derive(Clone, Copy)]
pub struct PIDConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

pub struct LoggerHandler {
    logger_status: AtomicBool,
    logger_time_sampling_ms: AtomicU32,
    pub log_tx_buffer: Channel<CriticalSectionRawMutex, LogData, LOG_BUFFER_SIZE>,
}

pub struct MotorHandler {
    current_pos: AtomicI32,
    current_speed: AtomicI32,
    current_commanded_pos: AtomicI32,
    current_commanded_speed: AtomicI32,
    current_commanded_pwm: AtomicI32,
    commanded_motor_speed: Channel<CriticalSectionRawMutex, MotorCommand, 16>,
    pos_pid: Mutex<CriticalSectionRawMutex, PosPIDConfig>,
    speed_pid: Mutex<CriticalSectionRawMutex, PIDConfig>,
    pub refresh_interval_us: u64,
    pub max_pwm_output_us: u64,
    pub max_speed_cps: i32,
    pub id: u8, 
}

pub struct Packet {
    pub data: [u8; USB_BUFFER_SIZE],
    pub len: usize,
}

pub struct LogData {
    pub seq: u8,
    pub dt: u32,
    pub values: [i32; 5],
}

/* --------------------------- Struct Implementation -------------------------- */
impl LoggerHandler {
    pub const fn new() -> Self {
        Self {
            logger_status: AtomicBool::new(false),
            logger_time_sampling_ms: AtomicU32::new(10),
            log_tx_buffer: Channel::new(),
        }
    }

    pub fn set_logging_state(&self, state: bool) {
        self.logger_status.store(state, Ordering::Relaxed);
    }
    
    pub fn is_logging_active(&self) -> bool {
        return self.logger_status.load(Ordering::Relaxed);
    }
    
    pub fn set_logging_time_sampling(&self, time_sampling_ms: u64) {
       self.logger_time_sampling_ms.store(time_sampling_ms as u32, Ordering::Relaxed);
    }
    
    pub fn get_logging_time_sampling(&self) -> u64 {
        return self.logger_time_sampling_ms.load(Ordering::Relaxed) as u64;
    }
}

impl MotorHandler {
    pub const fn new(id: u8) -> Self {
        Self {
            current_pos: AtomicI32::new(0),
            current_speed: AtomicI32::new(0),
            current_commanded_pos: AtomicI32::new(0),
            current_commanded_speed: AtomicI32::new(0),
            current_commanded_pwm: AtomicI32::new(0),
            commanded_motor_speed: Channel::new(),
            pos_pid: Mutex::new(PosPIDConfig{ kp: 10.0, ki: 0.0, kd: 2.0, kp_speed: 1.0, ki_speed: 0.025, kd_speed: 0.0}),
            speed_pid: Mutex::new(PIDConfig{ kp: 1.0, ki: 0.1, kd: 2.0}),
            refresh_interval_us: 1000,
            max_pwm_output_us: 1000,
            max_speed_cps: 1130,
            id: id,
        }
    }

    pub fn set_current_pos(&self, pos: i32) {
        self.current_pos.store(pos, Ordering::Relaxed);
    }

    pub fn get_current_pos(&self) -> i32 {
        return self.current_pos.load(Ordering::Relaxed);
    }

    pub fn set_current_speed(&self, speed: i32) {
        self.current_speed.store(speed, Ordering::Relaxed);   
    }

    pub fn get_current_speed(&self) -> i32 {
        return self.current_speed.load(Ordering::Relaxed);
    }
    
    pub fn set_commanded_pos(&self, pos: i32) {
        self.current_commanded_pos.store(pos, Ordering::Relaxed);
    }

    pub fn get_commanded_pos(&self) -> i32 {
        return self.current_commanded_pos.load(Ordering::Relaxed);
    }

    pub fn set_commanded_speed(&self, speed: i32) {
        self.current_commanded_speed.store(speed, Ordering::Relaxed);  
    }

    pub fn get_commanded_speed(&self) -> i32 {
        return self.current_commanded_speed.load(Ordering::Relaxed);
    }

    pub fn set_motor_command(&self, speed: MotorCommand) {
        let _ = self.commanded_motor_speed.try_send(speed);
    }

    pub fn get_motor_command(&self) -> Option<MotorCommand> {
        return self.commanded_motor_speed.try_receive().ok();
    }

    pub fn set_commanded_pwm(&self, pwm: i32) {
        self.current_commanded_pwm.store(pwm, Ordering::Relaxed);   
    }

    pub fn get_commanded_pwm(&self) -> i32 {
        return self.current_commanded_pwm.load(Ordering::Relaxed);
    }

    pub async fn set_pos_pid(&self, config: PosPIDConfig) {
        let mut current = self.pos_pid.lock().await;
        *current = config;
    }

    pub async fn get_pos_pid(&self) -> PosPIDConfig {
        return *self.pos_pid.lock().await;
    }

    pub async fn set_speed_pid(&self, config: PIDConfig) {
        let mut current = self.speed_pid.lock().await;
        *current = config;
    }

    pub async fn get_speed_pid(&self) -> PIDConfig {
        return *self.speed_pid.lock().await;
    }
}

impl Packet {
    pub fn new() -> Self {
        Self { data: [0u8; 64], len: 0 }
    }
    pub fn push_bytes(&mut self, bytes: &[u8]) {
        let remain = 64 - self.len;
        let to_copy = bytes.len().min(remain);
        self.data[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
    }
}

impl LogData {
    pub fn pack_data(&self, out: &mut [u8]) {
        if out.len() < 26 { return; }

        out[0] = LOGGER_HEADER;            
        out[1] = self.seq;
        out[2..6].copy_from_slice(&self.dt.to_le_bytes());
        out[6..10].copy_from_slice(&self.values[0].to_le_bytes());
        out[10..14].copy_from_slice(&self.values[1].to_le_bytes());
        out[14..18].copy_from_slice(&self.values[2].to_le_bytes());
        out[18..22].copy_from_slice(&self.values[3].to_le_bytes());
        out[22..26].copy_from_slice(&self.values[4].to_le_bytes());
    }
}