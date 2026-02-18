/*
    DC Motor Resources
*/


use super::*;

/* --------------------------- Motor Command -------------------------- */
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

#[derive(PartialEq, Clone, Copy)]
pub enum ControlMode {
    Speed,
    Position,
    OpenLoop,
    Stop,
}

/* --------------------------- PID Config -------------------------- */
#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct PIDConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub i_limit: f32, // Symmetric for negative and positive
}

/* --------------------------- Motor Struct -------------------------- */
pub struct MotorHandler {
    current_pos: AtomicI32,
    current_speed: AtomicI32,
    current_commanded_pos: AtomicI32,
    current_commanded_speed: AtomicI32,
    current_commanded_pwm: AtomicI32,
    commanded_motor_speed: Channel<CriticalSectionRawMutex, MotorCommand, 16>,
    pos_pid: Mutex<CriticalSectionRawMutex, PIDConfig>,
    speed_pid: Mutex<CriticalSectionRawMutex, PIDConfig>,
    pub max_pwm_ticks: i32,
    pub max_speed_cps: i32,
    pub id: u8, 
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
            pos_pid: Mutex::new(DEFAULT_PID_POS_CONFIG),
            speed_pid: Mutex::new(DEFAULT_PID_SPEED_CONFIG),
            max_pwm_ticks: MOTOR_MAX_PWM_TICKS,
            max_speed_cps: MOTOR_CONTROL_MAX_SPEED_CPS,
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

    pub async fn set_pos_pid(&self, config: PIDConfig) {
        let mut current = self.pos_pid.lock().await;
        *current = config;
    }

    pub async fn get_pos_pid(&self) -> PIDConfig {
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