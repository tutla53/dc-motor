mod apps;
mod board;
mod basic_function;
mod logger;
mod config;
mod program;

use crate::apps::editor::initialized_editor;
use crate::basic_function::utility::safe_exit;
use crate::board::rpi::Pico;
use crate::board::rpi::CommandDef;
use crate::board::rpi::LogEntry;
use crate::basic_function::move_motor::Motor;
use crate::logger::fwlogger::Logger;
use crate::apps::cli::ReplCli;
use crate::apps::cli::ReplCommands;
use crate::apps::command_handler as CommandHandler;
use crate::program::initialize_script;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::sync::mpsc::Receiver;
use clap::Parser;
use rustyline::error::ReadlineError;
use colored::Colorize;

type BoardOutput = (Pico, Receiver<LogEntry>, HashMap<String, CommandDef>);
type SharedResponse = Arc<Mutex<HashMap<u8, Result<Vec<u8>, u8>>>>;
type ResourcesOutput = (SharedResources, HashMap<String, CommandDef>, Vec<String>, Vec<String>);

const MOTOR_ID: u8 = 0;

include!(concat!(env!("OUT_DIR"), "/generated_commands.rs"));

#[derive(Clone)]
pub struct SharedResources {
    pico: Arc<Mutex<Pico>>,
    m0: Arc<Mutex<Motor>>, 
    logger: Arc<Mutex<Logger>>,
}

impl SharedResources {
    fn new(motor_id: u8) -> Result<ResourcesOutput, Box<dyn std::error::Error>> {
        let (pico, log_rx, cmd_definitions) = Pico::new(env!("CONFIG_FILE"))?;

        let mut available_commands: Vec<String> = cmd_definitions.keys().cloned().collect();
        available_commands.sort();

        let mut available_routines: Vec<String> = PROGRAM_FUNCTIONS.iter().map(|s| s.to_string()).collect();
        available_routines.sort();

        let shared_pico = Arc::new(Mutex::new(pico));
        let m0 = Motor::new(Arc::clone(&shared_pico), motor_id);
        let logger = Logger::new(Arc::clone(&shared_pico), log_rx, m0.motor_id);

        let shared_m0 = Arc::new(Mutex::new(m0));
        let shared_logger = Arc::new(Mutex::new(logger));

        let shared = Self {
            pico: shared_pico, 
            m0: shared_m0, 
            logger: shared_logger,
        };

        initialize_script(shared.clone())?;
        Ok((shared, cmd_definitions, available_commands, available_routines))
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (shared, command_def, available_commands, available_routines) = SharedResources::new(MOTOR_ID)?;
    let mut editor = initialized_editor(&available_commands, &available_routines)?;
    let prompt = format!("{}> ", "rp2040".green());

    println!("Type 'run' to run the function, or 'exit' to quit.");

    loop {
        let readline = editor.readline(&prompt);
        match readline {
            Ok(line) => {
                let trimmed = line.trim();
                if trimmed.is_empty() { continue; }
                
                let _ = editor.add_history_entry(trimmed);
                let args: Vec<&str> = trimmed.split_whitespace().collect();

                match ReplCli::try_parse_from(&args) {
                    Ok(cli) => {
                        match cli.command {
                            ReplCommands::Dev { all, command_name, args } => {
                                CommandHandler::api_handler(
                                    Arc::clone(&shared.pico), 
                                    &available_commands,  
                                    &command_def,
                                    all, 
                                    command_name, 
                                    args
                                );
                            },
                            ReplCommands::Run { all, routine_name, args } => {
                                CommandHandler::run_program_handler(
                                    &available_routines,
                                    all, 
                                    routine_name, 
                                    args
                                );
                            },
                            ReplCommands::Exit => {
                                println!("Exiting loop...");
                                break;
                            }
                        }
                    }
                    Err(clap_error) => {
                        println!("{}", clap_error);
                    }
                }
            }
            Err(ReadlineError::Interrupted) | Err(ReadlineError::Eof) => {
                safe_exit(
                    Arc::clone(&shared.pico),
                    Arc::clone(&shared.m0), 
                    Arc::clone(&shared.logger),
                );
                println!("\nSession terminated.");
                break;
            }
            Err(err) => {
                println!("Error reading line: {:?}", err);
                break;
            }
        }
    }
    
    Ok(())
}