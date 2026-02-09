/*
    Firmware Logger Resources
*/

use super::*;

/* --------------------------- Loger Data -------------------------- */
pub struct LogData {
    pub seq: u8,
    pub dt: u32,
    pub values: [i32; 5],
}

impl LogData {
    pub fn pack_data(&self, out: &mut [u8]) {
        if out.len() < 26 { return; }

        out[0] = HEADER::LOGGER as u8;            
        out[1] = self.seq;
        out[2..6].copy_from_slice(&self.dt.to_le_bytes());
        out[6..10].copy_from_slice(&self.values[0].to_le_bytes());
        out[10..14].copy_from_slice(&self.values[1].to_le_bytes());
        out[14..18].copy_from_slice(&self.values[2].to_le_bytes());
        out[18..22].copy_from_slice(&self.values[3].to_le_bytes());
        out[22..26].copy_from_slice(&self.values[4].to_le_bytes());
    }
}

/* --------------------------- Logger Handle -------------------------- */
pub struct LoggerHandler {
    logger_status: AtomicBool,
    logger_time_sampling_ms: AtomicU32,
    pub log_tx_buffer: Channel<CriticalSectionRawMutex, LogData, LOG_BUFFER_SIZE>,
}

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