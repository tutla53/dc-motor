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

/* --------------------------- Motor Struct -------------------------- */
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
    pub max_pwm_output_us: i32,
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