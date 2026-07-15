
use crate::SharedResources;
use crate::basic_function::utility::wait_ms;

pub mod script;
pub mod macros;

use std::time::Duration;
use std::sync::OnceLock;

use crate::with_lock;
use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Speed;
use crate::basic_function::conversion::Acceleration;

// Global resources can be used only on for the script.rs
static SHARED: OnceLock<SharedResources> = OnceLock::new();

pub fn initialize_script(shared: SharedResources) -> Result<(), Box<dyn std::error::Error>> {
    SHARED.set(shared.clone()).map_err(|_| "Shared resources already initialized!")?;
    Ok(())
}