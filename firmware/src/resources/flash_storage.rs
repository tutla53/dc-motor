/*
    Flash Storage Resources
*/

use super::*;

pub type MyStorage = MapStorage<u8, Flash<'static, embassy_rp::peripherals::FLASH, Async, FLASH_SIZE>, NoCache>;
pub static STORAGE: Mutex<ThreadModeRawMutex, Option<MyStorage>> = Mutex::new(None);

pub async fn save_pid_config<T:Serialize> (key: u8, config: &T) {
    let mut mutex_guard = STORAGE.lock().await;
    if let Some(storage) = mutex_guard.as_mut() {
        let mut serialization_buf = [0u8; 64];
        let mut data_buffer = [0u8; 128];

        // Serialize f32 PIDConfig

        // TODO: ADD ERROR HANDLING
        let byte_data: &[u8] = postcard::to_slice(config, &mut serialization_buf).unwrap();

        // TODO: ADD ERROR HANDLING
        match storage.store_item(&mut data_buffer, &key, &byte_data).await {
            Ok(()) => {},
            Err(_) => {},
        }
    }
}

pub async fn load_pid_config<T: DeserializeOwned>(key: u8, default: T) -> T {
    let mut mutex_guard = STORAGE.lock().await;
    if let Some(storage) = mutex_guard.as_mut() {
        let mut data_buffer = [0u8; 128];

        match storage.fetch_item::<&[u8]>(&mut data_buffer, &key).await {
            Ok(Some(bytes)) => {
                return postcard::from_bytes::<T>(bytes).unwrap_or(default);
            }
            _ => return default,
        }
    }
    default
}