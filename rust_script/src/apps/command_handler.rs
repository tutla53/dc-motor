
use super::*;

pub fn api_handler(
    pico: Arc<Mutex<Pico>>,
    available_commands: &Vec<String>, 
    command_def: &HashMap<String, CommandDef>, 
    all: bool, command_name: Option<String>, 
    args: Vec<String>
) {
    if all {
        println!("{}", "Available hardware API commands:".cyan());
        for cmd in available_commands {
            println!("  dev {}", cmd);
        }        
    }
    else if let Some(cmd_name) = command_name {

        if let Some(cmd_def) = command_def.get(&cmd_name) {
            if args.len() != cmd_def.args.len() {
                println!("Error: Expected {} args, got {}.", cmd_def.args.len(), args.len());
                println!("Expected arguments: {:?}", cmd_def.args);
                return;
            }

            let mut payload = Vec::new();
            let mut parse_failed = false;

            for (i, (_arg_name, ty)) in cmd_def.args.iter().enumerate() {
                let value_str = &args[i];
                let res = match ty.as_str() {
                    "u8" => value_str.parse::<u8>().map(|v| payload.push(v)).map_err(|_| ()),
                    "i32" => value_str.parse::<i32>().map(|v| payload.extend_from_slice(&v.to_le_bytes())).map_err(|_| ()),
                    "f32" => value_str.parse::<f32>().map(|v| payload.extend_from_slice(&v.to_le_bytes())).map_err(|_| ()),
                    "u64" => value_str.parse::<u64>().map(|v| payload.extend_from_slice(&v.to_le_bytes())).map_err(|_| ()),
                    _ => Err(()),
                };

                if res.is_err() {
                    println!("Error parsing parameter '{}' as type '{}'", value_str, ty);
                    parse_failed = true;
                    break;
                }
            }

            if parse_failed { return; }

            println!("Sending: {} (Op: {}, Payload: {:?})", cmd_name, cmd_def.op, payload);

            if let Ok(mut pico) = pico.lock() {
                match pico.execute_command(cmd_def.op, &payload) {
                    Ok(Some(response_bytes)) => {
                        println!("Success! Response Bytes: {:?}", response_bytes);
                        
                        let mut offset = 0;
                        for (field_name, ty) in &cmd_def.ret {
                            match ty.as_str() {
                                "u8" if offset < response_bytes.len() => {
                                    println!("  {}: {}", field_name, response_bytes[offset]);
                                    offset += 1;
                                }
                                "i32" if offset + 4 <= response_bytes.len() => {
                                    let val = i32::from_le_bytes(response_bytes[offset..offset+4].try_into().unwrap());
                                    println!("  {}: {}", field_name, val);
                                    offset += 4;
                                }
                                "f32" if offset + 4 <= response_bytes.len() => {
                                    let val = f32::from_le_bytes(response_bytes[offset..offset+4].try_into().unwrap());
                                    println!("  {}: {}", field_name, val);
                                    offset += 4;
                                }
                                _ => {}
                            }
                        }
                    }
                    Ok(_) => println!("Success (No return data expected)."),
                    Err(e) => println!("{}", e),
                }
            }
        } else {
            println!("Command '{}' not found in TOML.", cmd_name);
        }
    }
}

pub fn run_program_handler(
    available_routines: &Vec<String>, 
    all: bool, 
    routine_name: Option<String>, 
    args: Vec<String>
) {
    if all {
        println!("{}", "Available Program Script:".cyan());
        for cmd in available_routines {
            println!("  run {}", cmd);
        } 
    }
    else if let Some(name) = routine_name {
        if let Some(expected_len) = crate::get_routine_arg_count(&name) {
            if args.len() != expected_len {
                println!(
                    "{} - Routine '{}' expects {} arguments, but you provided {}.", 
                    "Argument Error".bright_red().bold(), 
                    name, 
                    expected_len, 
                    args.len()
                );
                return;
            }
        } else {
            println!("{} - Routine '{}' not found.", "Execution Error".bright_red().bold(), name);
            return;
        }

        let str_args: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
        if let Err(e) = execute_program_routine(&name, &str_args) {
            println!("{} - {}", "Execution Error".bright_red().bold(), e);
        }
    }
}