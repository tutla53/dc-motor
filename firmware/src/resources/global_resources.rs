/* 
* Global Resources 
*/

// Library
use defmt_rtt as _;
use panic_probe as _;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/* --------------------------- Global Variables -------------------------- */
pub static MOTOR_0: MotorState = MotorState::new(0);
pub static LOGGER: LoggerState = LoggerState::new();
pub static EVENT: EventHandler = EventHandler::new();

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
    MotorMoveDone(i32),
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

pub struct LoggerState {
    logger_run: Mutex<CriticalSectionRawMutex, bool>,
    logger_time_sampling: Mutex<CriticalSectionRawMutex, u64>,
    log_mask: Mutex<CriticalSectionRawMutex, u32>,
    log_request_queue: Channel<CriticalSectionRawMutex, bool, 1024>,
}

pub struct MotorState {
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
    id: i32, 
}

pub struct EventHandler {
    motor_event: Channel<CriticalSectionRawMutex, EventList, 256>,
}

/* --------------------------- Struct Implementation -------------------------- */
impl LoggerState {
    pub const fn new() -> Self {
        Self {
            logger_run: Mutex::new(false),
            logger_time_sampling: Mutex::new(10),
            log_mask: Mutex::new(0),
            log_request_queue: Channel::new(),
        }
    }

    pub async fn set_logging_state(&self, state: bool) {
        let mut logging = self.logger_run.lock().await;
        *logging = state;
    }
    
    pub async fn is_logging_active(&self, ) -> bool {
        return *self.logger_run.lock().await;
    }
    
    pub async fn set_logging_time_sampling(&self, time_sampling_ms: u64) {
        let mut current = self.logger_time_sampling.lock().await;
        *current = time_sampling_ms;
    }
    
    pub async fn get_logging_time_sampling(&self) -> u64 {
        return *self.logger_time_sampling.lock().await;
    }
    
    pub async fn get_log_mask(&self) -> u32 {
        return *self.log_mask.lock().await;
    }
    
    pub async fn set_log_mask(&self, log_mask: u32) {
        let mut current = self.log_mask.lock().await;
        *current = log_mask;
    }

    pub fn add_log_request(&self, cmd: bool) {
        let _ = self.log_request_queue.try_send(cmd);
    }

    pub async fn wait_for_log_request(&self) -> bool {
        return self.log_request_queue.receive().await;
    }
}

impl MotorState {
    pub const fn new(id: i32) -> Self {
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

    pub fn get_id(&self) -> i32 {
        return self.id;
    }
}

impl EventHandler {
    pub const fn new() -> Self {
        Self {
            motor_event: Channel::new(),
        }
    }
    pub fn set_event_item(&self, cmd: EventList) {
        let _ = self.motor_event.try_send(cmd);
    }

    pub async fn get_event_item(&self) -> EventList {
        return self.motor_event.receive().await;
    }
}