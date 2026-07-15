use crate::execute_program_routine;
use crate::board::rpi::Pico;
use crate::board::rpi::CommandDef;

use clap::{Parser, Subcommand};
use rustyline::completion::{Completer, Pair};
use rustyline::hint::{Hinter};
use rustyline::{Context, Helper};
use rustyline::highlight::Highlighter;
use rustyline::validate::Validator;
use rustyline::{Config, Editor};
use rustyline::history::DefaultHistory;
use colored::Colorize;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub mod command_handler;
pub mod cli;
pub mod editor;