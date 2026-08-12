/*
* plotter hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::config::motor_config;
use crate::plotter::plotter_config::Canvas;
use crate::plotter::plotter_config::ChartConfig;
use crate::plotter::plotter_config::ColorConfig;
use crate::plotter::plotter_config::FontSize;
use crate::plotter::plotter_config::Margin;
use crate::plotter::plotter_config::SERIES_COLORS;
use crate::plotter::plotter_config::TIMESTAMP_INDEX;
use crate::plotter::plotter_config::Y_AXIS_OFFSET;
use crate::tool::csv_processing::CsvProcessing;

/* ------------------------ Library --------------------------- */
use plotters::element::DashedPathElement;
use plotters::prelude::*;
use plotters::series::DashedLineSeries;

/* --------------------- Declare Modules ---------------------- */
pub mod plot;
pub mod plotter_config;
