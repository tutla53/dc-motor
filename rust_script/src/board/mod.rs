/*
* board hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::BoardOutput;
use crate::SharedResponse;

/* ------------------------ Library --------------------------- */
use colored::Colorize;
use serialport::SerialPort;
use serialport::SerialPortType;
use std::collections::HashMap;
use std::fs;
use std::io::Read;
use std::io::Write;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use std::time::Instant;

/* --------------------- Declare Modules ---------------------- */
pub mod rpi;
