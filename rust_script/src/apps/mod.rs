/*
* apps hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::board::rpi::CommandDef;
use crate::board::rpi::Pico;
use crate::execute_program_routine;

/* ------------------------ Library --------------------------- */
use clap::{Parser, Subcommand};
use colored::Colorize;
use rustyline::completion::{Completer, Pair};
use rustyline::highlight::Highlighter;
use rustyline::hint::Hinter;
use rustyline::history::DefaultHistory;
use rustyline::validate::Validator;
use rustyline::{Config, Editor};
use rustyline::{Context, Helper};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/* --------------------- Declare Modules ---------------------- */
pub mod cli;
pub mod command_handler;
pub mod editor;
