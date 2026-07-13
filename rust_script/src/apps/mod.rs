use crate::SHARED;
use crate::basic_function::utility::wait_ms;
use crate::basic_function::conversion::Position;
use crate::basic_function::conversion::Speed;
use crate::basic_function::conversion::Acceleration;

use clap::{Parser, Subcommand};
use rustyline::completion::{Completer, Pair};
// use rustyline::Completer;
use rustyline::hint::{Hinter};
use rustyline::{Context, Helper};
use rustyline::highlight::Highlighter;
use rustyline::validate::Validator;
use rustyline::{Config, Editor};
use rustyline::history::DefaultHistory;
use colored::Colorize;
use std::time::Duration;

pub mod command_handler;
pub mod cli;
pub mod editor;