/*
* program hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::config::motor_config;
use crate::config::nonlinear;
use crate::tool::csv_processing::CsvProcessing;

/* ------------------------ Library --------------------------- */
use fixed::types::I16F16;
use fixed::types::I32F32;
use motor_control::PIDConfig;
use motor_control::PIDController;
use plotters::prelude::RGBColor;
use std::io::Error;
use std::io::ErrorKind;

/* --------------------- Declare Modules ---------------------- */
pub mod model;
