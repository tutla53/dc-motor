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
    pub fn pack_data(&self) -> Result<Packet, PacketError> {
        let mut buffer = Packet::new();

        buffer
            .push(UsbHeader::Logger as u8)?
            .push(self.seq)?
            .push(self.dt)?;

        for value in self.values {
            buffer.push(value)?;
        }

        debug_assert_eq!(buffer.len(), LOG_PACKET_SIZE);

        Ok(buffer)
    }
}

/* --------------------------- Logger Handle -------------------------- */
pub struct LoggerHandler {
    logger_status: AtomicBool,
    logger_time_sampling_ms: AtomicU32,
    motor_id: AtomicU8,
    pub log_tx_buffer: Channel<CriticalSectionRawMutex, LogData, LOG_BUFFER_SIZE>,
}

impl LoggerHandler {
    pub const fn new() -> Self {
        Self {
            logger_status: AtomicBool::new(false),
            logger_time_sampling_ms: AtomicU32::new(10),
            motor_id: AtomicU8::new(255),
            log_tx_buffer: Channel::new(),
        }
    }

    pub fn set_logging_state(&self, state: bool) {
        self.logger_status.store(state, Ordering::Relaxed);
    }

    pub fn is_logging_active(&self) -> bool {
        self.logger_status.load(Ordering::Relaxed)
    }

    pub fn set_logging_time_sampling(&self, time_sampling_ms: u64) {
        self.logger_time_sampling_ms
            .store(time_sampling_ms as u32, Ordering::Relaxed);
    }

    pub fn get_logging_time_sampling(&self) -> u64 {
        self.logger_time_sampling_ms.load(Ordering::Relaxed) as u64
    }

    pub fn set_motor_id(&self, motor_id: u8) {
        self.motor_id.store(motor_id, Ordering::Relaxed);
    }

    pub fn get_motor_id(&self) -> Option<usize> {
        match self.motor_id.load(Ordering::Relaxed) {
            255 => None,
            id => Some(id as usize),
        }
    }
}
