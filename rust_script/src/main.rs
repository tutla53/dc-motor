mod apps;
mod board;
mod basic_function;
mod logger;
mod config;

use crate::apps::editor::initialized_editor;
use crate::basic_function::utility::safe_exit;
use crate::board::rpi::Pico;
use crate::basic_function::move_motor::Motor;
use crate::logger::fwlogger::Logger;
use crate::apps::cli::ReplCli;
use crate::apps::cli::ReplCommands;
use crate::apps::command_handler as CommandHandler;

use std::sync::{Arc, Mutex, OnceLock};
use clap::Parser;
use rustyline::error::ReadlineError;
use colored::Colorize;

const MOTOR_ID: u8 = 0;
pub static SHARED: OnceLock<SharedResources> = OnceLock::new();

include!(concat!(env!("OUT_DIR"), "/generated_commands.rs"));

pub struct SharedResources {
    pico: Arc<Mutex<Pico>>,
    m0: Arc<Mutex<Motor>>, 
    logger: Arc<Mutex<Logger>>,
    available_commands: Vec<String>,
}

impl SharedResources {
    fn new(motor_id: u8) -> Result<Self, Box<dyn std::error::Error>> {
        let (pico, log_rx) = Pico::new(env!("CONFIG_FILE"))?;
        let mut available_commands: Vec<String> = pico.cmd_definitions.keys().cloned().collect();
        available_commands.sort();

        let shared_pico = Arc::new(Mutex::new(pico));
        let m0 = Motor::new(Arc::clone(&shared_pico), motor_id);
        let logger = Logger::new(Arc::clone(&shared_pico), log_rx, m0.motor_id);

        let shared_m0 = Arc::new(Mutex::new(m0));
        let shared_logger = Arc::new(Mutex::new(logger));
        
        Ok(SharedResources {
            pico: shared_pico, 
            m0: shared_m0, 
            logger: shared_logger,
            available_commands,
            }
        )
    }

    fn initialized_resources(motor_id: u8) -> Result<(), Box<dyn std::error::Error>> {
        let resources = Self::new(motor_id)?;
        SHARED.set(resources).map_err(|_| "Shared resources already initialized!")?;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let _ = SharedResources::initialized_resources(MOTOR_ID)?;
    let mut editor = initialized_editor()?;

    let prompt = format!("{}> ", "rp2040".green());
    println!("Type 'move' to run the function, or 'exit' to quit.");

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
                            ReplCommands::Move => {
                                CommandHandler::move_motor_handler()?;
                            }
                            ReplCommands::Dev { all, command_name, args } => {
                                CommandHandler::api_handler(all, command_name, args);
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
                safe_exit();
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