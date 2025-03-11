
use {
    embassy_sync::{
        mutex::Mutex,
        channel::Channel,
        blocking_mutex::raw::CriticalSectionRawMutex,
    },
    {defmt_rtt as _, panic_probe as _},
};

static LOGGER_CONTROL: Channel<CriticalSectionRawMutex, LogMask, 1024> = Channel::new();
static LOGGER_RUN: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(false);
static LOGGER_TIME_SAMPLING: Mutex<CriticalSectionRawMutex, u64> = Mutex::new(10);
static CURRENT_POS: Mutex<CriticalSectionRawMutex, i32> = Mutex::new(0);
static CURRENT_SPEED: Mutex<CriticalSectionRawMutex, i32> = Mutex::new(0); // count/s
static COMMANDED_MOTOR_SPEED: Mutex<CriticalSectionRawMutex, CommandedSpeed> = Mutex::new(CommandedSpeed::Stop);
static KP: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.2);
static KI: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.05);
static KD: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(2.0);

pub struct LogMask {
    pub dt: u64,
    pub motor_speed:i32,
}

#[derive(Clone, Copy)]
pub enum CommandedSpeed {
    Speed(i32),
    Stop,
}

pub async fn set_logging_state(state: bool) {
    let mut logging = LOGGER_RUN.lock().await;
    *logging = state;
}

pub async fn is_logging_active() -> bool {
    return *LOGGER_RUN.lock().await;
}

pub async fn set_logging_time_sampling(time_sampling_ms: u64) {
    let mut current = LOGGER_TIME_SAMPLING.lock().await;
    *current = time_sampling_ms;
}

pub async fn get_logging_time_sampling() -> u64 {
    return *LOGGER_TIME_SAMPLING.lock().await;
}

pub async fn send_logged_data(data: LogMask) {
    let _ = LOGGER_CONTROL.try_send(data);
}

pub async fn get_logged_data() -> LogMask {
    return LOGGER_CONTROL.receive().await;
}

pub async fn set_current_pos(motor_id: u8, pos: i32) {
    match motor_id {
        0 => {
            let mut current_pos = CURRENT_POS.lock().await;
            *current_pos = pos;
        },
        _ => {},
    }
}

pub async fn get_current_pos(motor_id: u8) -> i32 {
    match motor_id {
        0 => {
            return *CURRENT_POS.lock().await;
        },
        _ => {
            return 0;
        },
    }
}

pub async fn set_current_speed(motor_id: u8, speed: i32) {
    match motor_id {
        0 => {
            let mut current_speed = CURRENT_SPEED.lock().await;
            *current_speed = speed;    
        },
        _ => {},
    }
}

pub async fn get_current_speed(motor_id: u8) -> i32 {
    match motor_id {
        0 => {
            return *CURRENT_SPEED.lock().await;
        },
        _ => {
            return 0;
        },
    }
}

pub async fn set_commanded_speed(motor_id: u8, speed: CommandedSpeed) {
    match motor_id {
        0 => {
            let mut current_speed = COMMANDED_MOTOR_SPEED.lock().await;
            *current_speed = speed;            
        },
        _ => {},
    }
}

pub async fn get_commanded_speed(motor_id: u8) -> CommandedSpeed {
    match motor_id {
        0 => {
            return *COMMANDED_MOTOR_SPEED.lock().await;   
        },
        _ => {
            CommandedSpeed::Stop
        },
    }
}

pub async fn set_kp(motor_id: u8, kp: f32) {
    match motor_id {
        0 => {
            let mut current_kp = KP.lock().await;
            *current_kp = kp;
        },
        _ => {},
    }
}

pub async fn get_kp(motor_id: u8) -> f32 {
    match motor_id {
        0 => {
            return *KP.lock().await;   
        },
        _ => {
            return 0.0;
        },
    }
}

pub async fn set_ki(motor_id: u8, ki: f32) {
    match motor_id {
        0 => {
            let mut current_ki = KI.lock().await;
            *current_ki = ki;
        },
        _ => {},
    }
}

pub async fn get_ki(motor_id: u8) -> f32 {
    match motor_id {
        0 => {
            return *KI.lock().await;   
        },
        _ => {
            return 0.0;
        },
    }
}

pub async fn set_kd(motor_id: u8, kd: f32) {
    match motor_id {
        0 => {
            let mut current_kd = KD.lock().await;
            *current_kd = kd;
        },
        _ => {},
    }
}

pub async fn get_kd(motor_id: u8) -> f32 {
    match motor_id {
        0 => {
            return *KD.lock().await;   
        },
        _ => {
            return 0.0;
        },
    }
}