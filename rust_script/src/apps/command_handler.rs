
use super::*;

pub fn api_handler(all: bool, command_name: Option<String>, args: Vec<String>) {
    let shared = SHARED.get().expect("Shared resources not initialized!");

    if all {
        println!("{}", "Available hardware API commands:".cyan());
        for cmd in &shared.available_commands {
            println!("  dev {}", cmd);
        }        
    }
    else if let Some(cmd_name) = command_name {
        let mut pico_guard = shared.pico.lock().unwrap();
        if let Some(cmd_def) = pico_guard.cmd_definitions.get(&cmd_name).cloned() {
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

            let expect_response = !cmd_def.ret.is_empty();
            println!("Sending: {} (Op: {}, Payload: {:?})", cmd_name, cmd_def.op, payload);

            match pico_guard.execute_command(cmd_def.op, &payload, expect_response) {
                Ok(Some(response_bytes)) => {
                    println!("Success! Response Bytes: {:?}", response_bytes);
                    
                    let mut offset = 0;
                    for (field_name, ty) in &cmd_def.ret {
                        match ty.as_str() {
                            "u8" if offset + 1 <= response_bytes.len() => {
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
                Ok(None) => println!("Success (No return data expected)."),
                Err(e) => println!("Hardware Error: {}", e),
            }
        } else {
            println!("Command '{}' not found in TOML.", cmd_name);
        }
    }
}

pub fn move_motor_handler() -> Result<(), Box<dyn std::error::Error>> {
    let shared = SHARED.get().expect("Shared resources not initialized!");
    let m0 = shared.m0.lock().unwrap();
    let mut logger = shared.logger.lock().unwrap();

    let current_pos = m0.get_motor_pos()?;
    println!("Initial Pos: {} count, {:.2} rotation", current_pos.count, current_pos.rotation);

    let target_rotation = if current_pos.rotation > 3.0 { 0.0 } else { 20.0 };
    let log_mask = 5;
    let time_sampling = 1;

    m0.clear_motor_event();

    logger.start(log_mask, time_sampling);
    wait_ms(300);

    m0.move_motor_pos_trapezoid(
        Position::from_rotation(target_rotation), 
        Speed::from_rpm(1000.0), 
        Acceleration::from_cps_sq(10_000),
    );
    
    m0.wait_move_done(Duration::from_secs(20))?;
    
    wait_ms(300);
    logger.stop();
    wait_ms(300);
    m0.stop_motor();
    
    let current_pos = m0.get_motor_pos()?;
    println!("Final Pos: {} count, {:.2} rotation", current_pos.count, current_pos.rotation);
    
    Ok(())
}