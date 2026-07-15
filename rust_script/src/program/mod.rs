/*
* program hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::SharedResources;
use crate::basic_function::conversion::Acceleration;
use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Speed;
use crate::basic_function::utility::wait_ms;
use crate::with_lock;

/* ------------------------ Library --------------------------- */
use std::sync::OnceLock;
use std::time::Duration;

/* --------------------- Declare Modules ---------------------- */
pub mod macros;
pub mod script;

// Global resources can be used only on for the script.rs
static SHARED: OnceLock<SharedResources> = OnceLock::new();

pub fn initialize_script(shared: SharedResources) -> Result<(), Box<dyn std::error::Error>> {
    SHARED
        .set(shared.clone())
        .map_err(|_| "Shared resources already initialized!")?;
    Ok(())
}
