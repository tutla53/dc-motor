/*
* Resources Hub
*/

/* --------------------------- Library -------------------------- */
use defmt_rtt as _;
use panic_probe as _;

use crate::StorageType;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use serde::Serialize;
use serde::de::DeserializeOwned;

/* --------------------------- Declare Modules -------------------------- */
pub mod storage_handler;

pub use storage_handler::*;
