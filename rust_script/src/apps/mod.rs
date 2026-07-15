/*
* apps hub
*/

/* ------------------------- Crate ---------------------------- */
use crate::board::rpi::CommandDef;
use crate::board::rpi::Pico;
use crate::execute_program_routine;

/* ------------------------ Library --------------------------- */
use clap::Parser;
use clap::Subcommand;
use colored::Colorize;
use rustyline::Config;
use rustyline::Context;
use rustyline::Editor;
use rustyline::Helper;
use rustyline::completion::Completer;
use rustyline::completion::Pair;
use rustyline::highlight::Highlighter;
use rustyline::hint::Hinter;
use rustyline::history::DefaultHistory;
use rustyline::validate::Validator;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Mutex;

/* --------------------- Declare Modules ---------------------- */
pub mod cli;
pub mod command_handler;
pub mod editor;
