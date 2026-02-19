/*
    Flash Storage Resources
*/

use super::*;

pub type MyStorage = MapStorage<u8, Flash<'static, embassy_rp::peripherals::FLASH, Async, FLASH_SIZE>, NoCache>;
pub static STORAGE: Mutex<ThreadModeRawMutex, Option<MyStorage>> = Mutex::new(None);

/* ------- List of Available Config which can be Saved on Flash Storage --------- */
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum ConfigType {
    SpeedPID = 0,
    PositionPID = 1,
}

/* --------------------------- Flash Storage -------------------------- */
fn get_flash_key(motor_id: u8, config: ConfigType) -> u8 {
    (motor_id << 4) | (config as u8)
}

pub async fn save_config<T:Serialize> (motor_id: u8, config_type: ConfigType, config: &T) -> Result<(), ()>{
    let key = get_flash_key(motor_id, config_type);
    
    let mut mutex_guard = STORAGE.lock().await;
    if let Some(storage) = mutex_guard.as_mut() {
        let mut serialization_buf = [0u8; 64];
        let mut data_buffer = [0u8; 128];

        let byte_data: &[u8] = postcard::to_slice(config, &mut serialization_buf).unwrap();

        match storage.store_item(&mut data_buffer, &key, &byte_data).await {
            Ok(()) => {
                return Ok(());
            },
            Err(_) => {
                return Err(());
            },
        }
    }
    Err(())
}

pub async fn load_config<T: DeserializeOwned + Serialize>(motor_id: u8, config_type: ConfigType, default: T) -> T {
    if let Ok(result) = fetch(motor_id, config_type).await {
        return result;
    }
    else {
        let _ = save_config(motor_id, config_type, &default).await;
        return default;
    }
}

async fn fetch<T: DeserializeOwned>(motor_id: u8, config_type: ConfigType) -> Result<T, ()> {
    let key = get_flash_key(motor_id, config_type);

    let mut mutex_guard = STORAGE.lock().await;
    if let Some(storage) = mutex_guard.as_mut() {
        let mut data_buffer = [0u8; 128];

        match storage.fetch_item::<&[u8]>(&mut data_buffer, &key).await {
            Ok(Some(bytes)) => {
                if let Ok(result) = postcard::from_bytes::<T>(bytes) {
                    return Ok(result);
                }
                else {
                    return Err(());
                }
            }
            _ => return Err(()),
        }
    }
    Err(())
}