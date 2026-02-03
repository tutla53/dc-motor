/* 
* Global Resources 
*/

// Library
use core::str;
use defmt_rtt as _;
use panic_probe as _;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/* --------------------------- Variables -------------------------- */
pub const USB_BUFFER_SIZE: usize = 64;
pub const EVENT_CHANNEL_SIZE: usize = 64;
pub const DATA_CHANNEL_SIZE: usize = 64;
pub const LOG_BUFFER_SIZE: usize = 1000;

/* --------------------------- Handlers-------------------------- */
pub static MOTOR_0: MotorHandler = MotorHandler::new(0);
pub static LOGGER: LoggerHandler = LoggerHandler::new();

/* --------------------------- Channels and StaticCell -------------------------- */
pub static USB_TX_CHANNEL: Channel<CriticalSectionRawMutex, Packet, USB_BUFFER_SIZE> = Channel::new();
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, EventList, EVENT_CHANNEL_SIZE> = Channel::new();
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, Packet, DATA_CHANNEL_SIZE> = Channel::new();

/* --------------------------- ENUM -------------------------- */
#[derive(Clone, Copy)]
pub enum Shape {
    Step(i32),
    Trapezoidal(f32, f32, f32),
}

#[derive(Clone, Copy)]
pub enum MotorCommand {
    SpeedControl(Shape),
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
    logger_status: Mutex<CriticalSectionRawMutex, bool>,
    logger_time_sampling_ms: Mutex<CriticalSectionRawMutex, u64>,
    pub log_tx_buffer: Channel<CriticalSectionRawMutex, LogData, LOG_BUFFER_SIZE>,
}

pub struct MotorHandler {
    current_pos: Mutex<CriticalSectionRawMutex, i32>,
    current_speed: Mutex<CriticalSectionRawMutex, i32>,
    current_commanded_pos: Mutex<CriticalSectionRawMutex, i32>,
    current_commanded_speed: Mutex<CriticalSectionRawMutex, i32>,
    current_commanded_pwm: Mutex<CriticalSectionRawMutex, i32>,
    commanded_motor_speed: Mutex<CriticalSectionRawMutex, MotorCommand>,
    pos_pid: Mutex<CriticalSectionRawMutex, PosPIDConfig>,
    speed_pid: Mutex<CriticalSectionRawMutex, PIDConfig>,
    refresh_interval_us: u64,
    max_pwm_output_us: u64,
    max_speed_cps: i32,
    id: u8, 
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
            logger_status: Mutex::new(false),
            logger_time_sampling_ms: Mutex::new(10),
            log_tx_buffer: Channel::new(),
        }
    }

    pub async fn set_logging_state(&self, state: bool) {
        let mut logging = self.logger_status.lock().await;
        *logging = state;
    }
    
    pub async fn is_logging_active(&self, ) -> bool {
        return *self.logger_status.lock().await;
    }
    
    pub async fn set_logging_time_sampling(&self, time_sampling_ms: u64) {
        let mut current = self.logger_time_sampling_ms.lock().await;
        *current = time_sampling_ms;
    }
    
    pub async fn get_logging_time_sampling(&self) -> u64 {
        return *self.logger_time_sampling_ms.lock().await;
    }
}

impl MotorHandler {
    pub const fn new(id: u8) -> Self {
        Self {
            current_pos: Mutex::new(0),
            current_speed: Mutex::new(0),
            current_commanded_pos: Mutex::new(0),
            current_commanded_speed: Mutex::new(0),
            current_commanded_pwm: Mutex::new(0),
            commanded_motor_speed: Mutex::new(MotorCommand::Stop),
            pos_pid: Mutex::new(PosPIDConfig{ kp: 10.0, ki: 0.0, kd: 2.0, kp_speed: 1.0, ki_speed: 0.025, kd_speed: 0.0}),
            speed_pid: Mutex::new(PIDConfig{ kp: 1.0, ki: 0.1, kd: 2.0}),
            refresh_interval_us: 1000,
            max_pwm_output_us: 1000,
            max_speed_cps: 1130,
            id: id,
        }
    }

    pub async fn set_current_pos(&self, pos: i32) {
        let mut current_pos = self.current_pos.lock().await;
        *current_pos = pos;
    }

    pub async fn get_current_pos(&self) -> i32 {
        return *self.current_pos.lock().await;
    }

    pub async fn set_current_speed(&self, speed: i32) {
        let mut current_speed = self.current_speed.lock().await;
        *current_speed = speed;    
    }

    pub async fn get_current_speed(&self) -> i32 {
        return *self.current_speed.lock().await;
    }

    pub async fn set_motor_command(&self, speed: MotorCommand) {
        let mut current_speed = self.commanded_motor_speed.lock().await;
        *current_speed = speed;            
    }

    pub async fn get_motor_command(&self) -> MotorCommand {
        return *self.commanded_motor_speed.lock().await;
    }

    pub async fn set_pos_pid(&self, config: PosPIDConfig) {
        let mut current = self.pos_pid.lock().await;
        *current = config;
    }

    pub async fn get_pos_pid(&self) -> PosPIDConfig {
        return *self.pos_pid.lock().await;
    }

    pub async fn get_commanded_speed(&self) -> i32 {
        return *self.current_commanded_speed.lock().await;
    }
    
    pub async fn set_commanded_speed(&self, speed: i32) {
        let mut current_speed = self.current_commanded_speed.lock().await;
        *current_speed = speed;    
    }
    
    pub async fn get_commanded_pwm(&self) -> i32 {
        return *self.current_commanded_pwm.lock().await;
    }
    
    pub async fn set_commanded_pwm(&self, pwm: i32) {
        let mut current_pwm = self.current_commanded_pwm.lock().await;
        *current_pwm = pwm;    
    }

    pub async fn get_commanded_pos(&self) -> i32 {
        return *self.current_commanded_pos.lock().await;
    }
    
    pub async fn set_commanded_pos(&self, pos: i32) {
        let mut current_pos = self.current_commanded_pos.lock().await;
        *current_pos = pos;    
    }

    pub async fn set_speed_pid(&self, config: PIDConfig) {
        let mut current = self.speed_pid.lock().await;
        *current = config;
    }

    pub async fn get_speed_pid(&self) -> PIDConfig {
        return *self.speed_pid.lock().await;
    }

    pub fn get_refresh_interval_us(&self) -> u64 {
        return self.refresh_interval_us;
    }

    pub fn get_max_pwm_output_us(&self) -> u64 {
        return self.max_pwm_output_us;
    }

    pub fn get_max_speed_cps(&self) -> i32 {
        return self.max_speed_cps;
    }

    pub fn get_id(&self) -> u8 {
        return self.id;
    }
}

impl Packet {
    pub fn new() -> Self {
        Self { data: [0u8; 64], len: 0 }
    }

    pub fn from_str(s: &str) -> Self {
        let mut data = [0u8; 64];
        let bytes = s.as_bytes();
        let len = bytes.len().min(64);
        data[..len].copy_from_slice(&bytes[..len]);
        Self { data, len }
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
        out[0] = 0xFF;            
        out[1] = self.seq;
        out[2..6].copy_from_slice(&self.dt.to_le_bytes());
        for i in 0..5 {
            let start = 6 + (i * 4);
            out[start..start + 4].copy_from_slice(&self.values[i].to_le_bytes());
        }
    }
}