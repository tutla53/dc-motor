
use serialport::{SerialPort, SerialPortType};
use std::collections::HashMap;
use std::fs;
use std::io::{Read, Write};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use colored::Colorize;

pub mod rpi;