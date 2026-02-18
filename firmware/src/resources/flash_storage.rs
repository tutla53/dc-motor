/*
    Flash Storage Resources
*/

use super::*;

pub type MyStorage = MapStorage<u8, Flash<'static, embassy_rp::peripherals::FLASH, Async, FLASH_SIZE>, NoCache>;
pub static STORAGE: Mutex<ThreadModeRawMutex, Option<MyStorage>> = Mutex::new(None);

pub async fn save_pid_config(key: u8, config: &PIDConfig) {
    let mut mutex_guard = STORAGE.lock().await;
    if let Some(storage) = mutex_guard.as_mut() {
        let mut serialization_buf = [0u8; 64];
        let mut data_buffer = [0u8; 128];

        // Serialize f32 PIDConfig
        let byte_data: &[u8] = postcard::to_slice(config, &mut serialization_buf).unwrap();

        // Store it
        match storage.store_item(&mut data_buffer, &key, &byte_data).await {
            Ok(()) => {},
            Err(_) => {},
        }
    }
}

pub async fn load_pid_config(key: u8) -> PIDConfig {
    let mut mutex_guard = STORAGE.lock().await;
    if let Some(storage) = mutex_guard.as_mut() {
        let mut data_buffer = [0u8; 128];

        match storage.fetch_item::<&[u8]>(&mut data_buffer, &key).await {
            Ok(Some(bytes)) => {
                return postcard::from_bytes::<PIDConfig>(bytes).unwrap_or(DEFAULT_PID_SPEED_CONFIG);
            }
            _ => return DEFAULT_PID_SPEED_CONFIG,
        }
    }
    DEFAULT_PID_SPEED_CONFIG
}