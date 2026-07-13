

use super::*;

#[derive(serde::Deserialize, Debug)]
struct TomlConfig {
    project: Option<ProjectDef>,
    headers: Option<HashMap<String, u8>>,
    commands: Option<Vec<CommandDef>>,
}

#[derive(serde::Deserialize, Debug)]
struct ProjectDef {
    version: String,
}

#[derive(serde::Deserialize, Debug, Clone)]
pub struct CommandDef {
    pub command: String,
    pub op: u8,
    
    #[serde(default)]
    pub args: HashMap<String, String>,

    #[serde(default)]
    pub ret: HashMap<String, String>,
}

#[allow(unused)]
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub seq: u8,
    pub dt: u32,
    pub values: [i32; 5],
}

#[allow(unused)]
pub struct Pico {
    port_tx: Option<Box<dyn SerialPort + Send>>, 
    headers: HashMap<String, u8>,
    pub event_rx: Receiver<String>,
    shared_responses: Arc<Mutex<HashMap<u8, Option<Vec<u8>>>>>,
    pub shared_events: Arc<Mutex<HashMap<u8, u8>>>, 
    pub sim_mode: bool,
    pub cmd_definitions: HashMap<String, CommandDef>,
}

impl Pico {
    pub fn new(toml_path: &str) -> Result<(Self, Receiver<LogEntry>), Box<dyn std::error::Error>> {
        let toml_content = fs::read_to_string(toml_path).unwrap_or_default();
        let config: TomlConfig = toml::from_str(&toml_content)?;
        let headers = config.headers.unwrap_or_default();
        let toml_version = config.project.map(|p| p.version).unwrap_or_else(|| "0.0.0".to_string());

        let mut response_sizes = HashMap::new();
        let mut cmd_definitions = HashMap::new();

        if let Some(cmds) = config.commands {
            for cmd in cmds {
                let size: usize = cmd.ret.values().map(|t| match t.as_str() {
                    "u8" => 1, "u16" => 2, "u32" | "i32" | "f32" => 4, "u64" => 8, _ => 0
                }).sum();
                response_sizes.insert(cmd.op, size);

                cmd_definitions.insert(cmd.command.clone(), cmd.clone());
            }
        }

        let ports = serialport::available_ports()?;
        let mut selected_port = None;
        for port in ports {
            if let SerialPortType::UsbPort(usb_info) = &port.port_type {
                if let Some(sn) = &usb_info.serial_number {
                    if sn.starts_with("12345678") {
                        selected_port = Some(port.port_name);
                        break;
                    }
                }
            }
        }

        let sim_mode = selected_port.is_none();
        let (log_tx, log_rx) = mpsc::channel();
        let (event_tx, event_rx) = mpsc::channel();
        let shared_responses = Arc::new(Mutex::new(HashMap::new()));
        let shared_events = Arc::new(Mutex::new(HashMap::new())); 

        println!("---------------------------------------------------");
        
        let port_tx = if !sim_mode {
            let port_name = selected_port.unwrap();
            let port = serialport::new(port_name, 115_200)
                .timeout(Duration::from_millis(10)) 
                .open()?;
            
            if let Some(port_name) = port.name() {
                println!("Connected to: {}", port_name.bright_green().bold());
            }
            
            let port_rx = port.try_clone()?; 
            
            Self::start_listener(
                port_rx, 
                headers.clone(), 
                response_sizes,
                log_tx, 
                event_tx,
                Arc::clone(&shared_responses),
                Arc::clone(&shared_events), 
            );
            
            Some(port as Box<dyn SerialPort + Send>)
        } else {
            println!("{}", "Entering Simulation Mode".bright_yellow().bold());
            None
        };

        let mut pico = Self {
            port_tx,
            headers,
            event_rx,
            shared_responses,
            shared_events, 
            sim_mode,
            cmd_definitions,
        };

        if !pico.sim_mode {
            let fw_version: String = match pico.get_firmware_version() {
                Ok((major, minor, patch)) => {
                    format!("{}.{}.{}", major, minor, patch)
                }
                Err(_) => {
                    "0.0.0".to_string()
                }
            };
            
            println!("Firmware Version: {}", fw_version.bright_yellow().bold());

            if toml_version == fw_version {
                println!("TOML Version: {} {}", toml_version.bright_yellow().bold(), "(matched with the FW version)".bright_green().bold());
            }
            else {
                println!("TOML Version: {} {}", 
                    toml_version.bright_red().bold(), 
                    "(unmatched with the FW version)".bright_red().bold(),
                );
                println!("{}", "Entering Simulation Mode".bright_yellow().bold());
                pico.sim_mode = true;                
            }
        }

        println!("---------------------------------------------------\n");
        Ok((pico, log_rx))
    }

    pub fn execute_command(
        &mut self,
        op: u8,
        payload: &[u8],
        expect_response: bool,
    ) -> Result<Option<Vec<u8>>, String> {
        if self.sim_mode {
            if expect_response {
                return Ok(Some(vec![0; 64]));
            }
            return Ok(None);
        }

        if expect_response {
            let mut responses = self.shared_responses.lock().unwrap();
            responses.remove(&op);
        }

        let cmd_header = self.headers.get("COMMAND").copied().unwrap_or(0xFF);

        if let Some(ref mut port) = self.port_tx {
            let mut packet = vec![cmd_header, op];
            packet.extend_from_slice(payload);
            
            port.write_all(&packet).map_err(|e| e.to_string())?;
            port.flush().map_err(|e| e.to_string())?;
        }

        if expect_response {
            let timeout = Duration::from_secs(5);
            let start = Instant::now();
            
            loop {
                {
                    let mut responses = self.shared_responses.lock().unwrap();
                    if let Some(res) = responses.remove(&op) {
                        return match res {
                            Some(bytes) => Ok(Some(bytes)),
                            None => Err("Pico returned a runtime error flag for this command".to_string()),
                        };
                    }
                }
                
                if start.elapsed() > timeout {
                    return Err(format!("Timeout waiting for response to OP code: 0x{:02X}", op));
                }
                thread::sleep(Duration::from_millis(5));
            }
        }

        Ok(None)
    }

    fn start_listener(
        mut port: Box<dyn SerialPort + Send>,
        headers: HashMap<String, u8>,
        response_sizes: HashMap<u8, usize>,
        log_tx: mpsc::Sender<LogEntry>,
        event_tx: mpsc::Sender<String>,
        shared_responses: Arc<Mutex<HashMap<u8, Option<Vec<u8>>>>>,
        shared_events: Arc<Mutex<HashMap<u8, u8>>>, 
    ) {
        let log_header = headers.get("LOGGER").copied().unwrap_or(0xFD);
        let ev_header = headers.get("EVENT").copied().unwrap_or(0xFE);
        let cmd_header = headers.get("COMMAND").copied().unwrap_or(0xFF);
        
        thread::spawn(move || {
            let mut buffer: Vec<u8> = Vec::new();
            let mut temp_buf = [0u8; 1024];

            loop {
                if let Ok(bytes_read) = port.read(&mut temp_buf) {
                    if bytes_read > 0 {
                        buffer.extend_from_slice(&temp_buf[..bytes_read]);

                        while !buffer.is_empty() {
                            let header = buffer[0];

                            if header == log_header {
                                if buffer.len() >= 26 {
                                    let seq = buffer[1];
                                    let dt = u32::from_le_bytes(buffer[2..6].try_into().unwrap());
                                    
                                    let mut values = [0i32; 5];
                                    for i in 0..5 {
                                        let start = 6 + (i * 4);
                                        values[i] = i32::from_le_bytes(buffer[start..start + 4].try_into().unwrap());
                                    }

                                    let log_entry = LogEntry { seq, dt, values };
                                    let _ = log_tx.send(log_entry);
                                    buffer.drain(..26);
                                } else {
                                    break; 
                                }
                            }
                            else if header == ev_header {
                                if buffer.len() >= 3 {
                                    let ev_code = buffer[1];
                                    let m_id = buffer[2];
                                    
                                    shared_events.lock().unwrap().insert(m_id, ev_code);

                                    let event_msg = format!("MOVE_MOTOR_DONE:{} {}", m_id, ev_code); 
                                    let _ = event_tx.send(event_msg);
                                    buffer.drain(..3);
                                } else {
                                    break; 
                                }
                            }
                            else if header == cmd_header {
                                if buffer.len() >= 2 {
                                    let error_code = buffer[1];
                                    if error_code == 0 { 
                                        if buffer.len() >= 3 {
                                            let op_code = buffer[2];
                                            let payload_size = response_sizes.get(&op_code).copied().unwrap_or(0);
                                            let total_size = 3 + payload_size;

                                            if buffer.len() >= total_size {
                                                let payload = buffer[3..total_size].to_vec();
                                                shared_responses.lock().unwrap().insert(op_code, Some(payload));
                                                buffer.drain(..total_size);
                                            } else {
                                                break; 
                                            }
                                        } else {
                                            break;
                                        }
                                    } else {
                                        if buffer.len() >= 3 {
                                            let op_code = buffer[2];
                                            shared_responses.lock().unwrap().insert(op_code, None);
                                            buffer.drain(..3);
                                        } else {
                                            break; 
                                        }
                                    }
                                } else {
                                    break;
                                }
                            }
                            else {
                                buffer.drain(0..1);
                            }
                        }
                    }
                }
                thread::sleep(Duration::from_millis(1));
            }
        });
    }
}