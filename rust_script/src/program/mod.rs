/*
* program hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::LogMask;
use crate::SharedResources;
use crate::basic_function::conversion::Acceleration;
use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Pwm;
use crate::basic_function::conversion::Speed;
use crate::basic_function::utility::wait_ms;
use crate::config::motor_config;
use crate::plotter::plot;
use crate::plotter::plotter_config::TIMESTAMP_INDEX;
use crate::plotter::plotter_config::Y_AXIS_OFFSET;
use crate::program::macros::MutexExt;
use crate::simulation::model::ModelKind;
use crate::simulation::model::MotorSimulation;
use crate::tool::csv_processing::CsvProcessing;
use crate::try_lock;

/* ------------------------ Library --------------------------- */
use std::sync::Mutex;
use std::sync::MutexGuard;
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
