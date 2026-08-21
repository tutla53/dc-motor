/*
    DC Motor Resources
*/

use super::*;

/* --------------------------- Motor Command -------------------------- */
#[derive(Clone, Copy, PartialEq)]
pub enum Shape {
    Step(i32),
    Trapezoidal(I32F32, I32F32, I32F32),
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

/* --------------------------- Motor Struct -------------------------- */
pub struct MotorHandler {
    current_pos: AtomicI32,
    current_speed: AtomicI32,
    current_commanded_pos: AtomicI32,
    current_commanded_speed: AtomicI32,
    current_commanded_pwm: AtomicI32,
    motor_command: Channel<CriticalSectionRawMutex, MotorCommand, 16>,
    pos_pid: Mutex<CriticalSectionRawMutex, PIDConfig>,
    speed_pid: Mutex<CriticalSectionRawMutex, PIDConfig>,
    pub default_pos_pid: PIDConfig,
    pub default_speed_pid: PIDConfig,
    pub default_max_speed: i32,
    pub max_pwm_ticks: i32,
    pub max_speed_cps: AtomicI32,
    pub move_done: AtomicBool,
    pub id: u8,
    enable: PortableAtomicBool,
    enable_requested: PortableAtomicBool,
    disable_requested: PortableAtomicBool,
}

impl MotorHandler {
    pub const fn new(id: u8) -> Self {
        Self {
            current_pos: AtomicI32::new(0),
            current_speed: AtomicI32::new(0),
            current_commanded_pos: AtomicI32::new(0),
            current_commanded_speed: AtomicI32::new(0),
            current_commanded_pwm: AtomicI32::new(0),
            motor_command: Channel::new(),
            pos_pid: Mutex::new(DEFAULT_PID_POS_CONFIG),
            speed_pid: Mutex::new(DEFAULT_PID_SPEED_CONFIG),
            default_pos_pid: DEFAULT_PID_POS_CONFIG,
            default_speed_pid: DEFAULT_PID_SPEED_CONFIG,
            default_max_speed: DEFAULT_MOTOR_CONTROL_MAX_SPEED_CPS,
            max_pwm_ticks: MOTOR_MAX_PWM_TICKS,
            max_speed_cps: AtomicI32::new(DEFAULT_MOTOR_CONTROL_MAX_SPEED_CPS),
            move_done: AtomicBool::new(false),
            id,
            enable: PortableAtomicBool::new(false),
            enable_requested: PortableAtomicBool::new(false),
            disable_requested: PortableAtomicBool::new(false),
        }
    }

    pub fn set_current_pos(&self, pos: i32) {
        self.current_pos.store(pos, Ordering::Relaxed);
    }

    pub fn get_current_pos(&self) -> i32 {
        self.current_pos.load(Ordering::Relaxed)
    }

    pub fn set_current_speed(&self, speed: i32) {
        self.current_speed.store(speed, Ordering::Relaxed);
    }

    pub fn get_current_speed(&self) -> i32 {
        self.current_speed.load(Ordering::Relaxed)
    }

    pub fn set_commanded_pos(&self, pos: i32) {
        self.current_commanded_pos.store(pos, Ordering::Relaxed);
    }

    pub fn get_commanded_pos(&self) -> i32 {
        self.current_commanded_pos.load(Ordering::Relaxed)
    }

    pub fn set_commanded_speed(&self, speed: i32) {
        self.current_commanded_speed.store(speed, Ordering::Relaxed);
    }

    pub fn get_commanded_speed(&self) -> i32 {
        self.current_commanded_speed.load(Ordering::Relaxed)
    }

    pub fn try_set_motor_command(
        &self,
        speed: MotorCommand,
    ) -> Result<(), TrySendError<MotorCommand>> {
        self.motor_command.try_send(speed)
    }

    pub fn get_motor_command(&self) -> Option<MotorCommand> {
        self.motor_command.try_receive().ok()
    }

    pub fn set_commanded_pwm(&self, pwm: i32) {
        self.current_commanded_pwm.store(pwm, Ordering::Relaxed);
    }

    pub fn get_commanded_pwm(&self) -> i32 {
        self.current_commanded_pwm.load(Ordering::Relaxed)
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

    pub fn set_max_speed(&self, speed: i32) {
        self.max_speed_cps.store(speed, Ordering::Relaxed);
    }

    pub fn get_max_speed(&self) -> i32 {
        self.max_speed_cps.load(Ordering::Relaxed)
    }

    pub fn set_move_done(&self, status: bool) {
        self.move_done.store(status, Ordering::Relaxed);
    }

    pub fn get_move_done(&self) -> bool {
        self.move_done.load(Ordering::Relaxed)
    }

    pub fn is_motion_enabled(&self) -> bool {
        self.enable.load(Ordering::SeqCst) && !self.disable_requested.load(Ordering::SeqCst)
    }

    pub fn is_motor_enabled(&self) -> bool {
        self.enable.load(Ordering::SeqCst)
    }

    pub fn request_motor_enable(&self) {
        if !self.is_motor_enabled() || self.disable_requested.load(Ordering::SeqCst) {
            self.enable_requested.store(true, Ordering::SeqCst);
        }
    }

    pub fn request_motor_disable(&self) {
        self.enable_requested.store(false, Ordering::SeqCst);
        self.disable_requested.store(true, Ordering::SeqCst);
    }

    pub fn take_enable_request(&self) -> bool {
        take_request(&self.enable_requested)
    }

    pub fn take_disable_request(&self) -> bool {
        take_request(&self.disable_requested)
    }

    pub fn set_motor_enabled(&self, status: bool) {
        self.enable.store(status, Ordering::SeqCst);
    }
}

fn take_request(request: &PortableAtomicBool) -> bool {
    loop {
        if !request.load(Ordering::SeqCst) {
            return false;
        }

        if request
            .compare_exchange(true, false, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
        {
            return true;
        }
    }
}
