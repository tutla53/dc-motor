use serialport::{SerialPort, SerialPortType};
use std::collections::HashMap;
use std::fs;
use std::io::{Read, Write};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

// ---------------------------------------------------------
// Runtime TOML Parsing structs
// ---------------------------------------------------------
#[derive(serde::Deserialize, Debug)]
struct TomlConfig {
    headers: Option<HashMap<String, u8>>,
    commands: Option<Vec<CommandDef>>,
}

#[derive(serde::Deserialize, Debug)]
struct CommandDef {
    op: u8,
    #[serde(default)]
    ret: HashMap<String, String>,
}

#[derive(Debug, Clone)]
pub struct LogEntry {
    pub seq: u8,
    pub dt: u32,
    pub values: [i32; 5],
}

// ---------------------------------------------------------
// Core Pico Struct
// ---------------------------------------------------------
pub struct Pico {
    port_tx: Option<Box<dyn SerialPort>>, 
    headers: HashMap<String, u8>,
    pub log_rx: Receiver<LogEntry>,
    pub event_rx: Receiver<String>,
    shared_responses: Arc<Mutex<HashMap<u8, Option<Vec<u8>>>>>,
    shared_events: Arc<Mutex<HashMap<u8, u8>>>, 
    pub sim_mode: bool,
}

impl Pico {
    pub fn new(toml_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let toml_content = fs::read_to_string(toml_path).unwrap_or_default();
        let config: TomlConfig = toml::from_str(&toml_content)?;
        let headers = config.headers.unwrap_or_default();
        
        let mut response_sizes = HashMap::new();
        if let Some(cmds) = config.commands {
            for cmd in cmds {
                let size: usize = cmd.ret.values().map(|t| match t.as_str() {
                    "u8" => 1, "u16" => 2, "u32" | "i32" | "f32" => 4, "u64" => 8, _ => 0
                }).sum();
                response_sizes.insert(cmd.op, size);
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

        let port_tx = if !sim_mode {
            let port_name = selected_port.unwrap();
            let port = serialport::new(port_name, 115_200)
                .timeout(Duration::from_millis(10)) 
                .open()?;
            
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
            
            Some(port)
        } else {
            None
        };

        Ok(Self {
            port_tx,
            headers,
            log_rx,
            event_rx,
            shared_responses,
            shared_events, 
            sim_mode,
        })
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
            let timeout = Duration::from_secs(2);
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

    /// Clears any pending move-done status flags for a specific motor
    pub fn clear_motor_event(&self, motor_id: u8) {
        if let Ok(mut events) = self.shared_events.lock() {
            events.remove(&motor_id);
        }
    }

    /// Blocks the calling thread until the requested motor finishes its movement loop
    pub fn wait_move_done(&self, motor_id: u8, timeout: Duration) -> Result<u8, String> {
        if self.sim_mode {
            thread::sleep(Duration::from_millis(500));
            return Ok(0);
        }

        let start = Instant::now();
        loop {
            {
                let mut events = self.shared_events.lock().unwrap();
                if let Some(ev_code) = events.remove(&motor_id) {
                    return Ok(ev_code);
                }
            }
            
            if start.elapsed() > timeout {
                return Err(format!("Timeout waiting for motor {} to finish moving", motor_id));
            }
            thread::sleep(Duration::from_millis(10));
        }
    }

    fn start_listener(
        mut port: Box<dyn SerialPort>,
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

// ---------------------------------------------------------
// Firmware Logger Implementation
// ---------------------------------------------------------
const N_ENUM: usize = 5;

const LOG_MASK_NAMES: [&str; N_ENUM] = [
    "Motor_Position(rotation)",
    "Motor_Speed(RPM)",
    "Commanded_Position(rotation)",
    "Commanded_Speed(RPM)",
    "Commanded_PWM",
];

const ROTATION_PER_PULSE: f64 = 0.001; 

const SCALE_OFFSET_MOTOR: [[f64; 2]; N_ENUM] = [
    [ROTATION_PER_PULSE, 0.0],
    [60.0 * ROTATION_PER_PULSE, 0.0],
    [ROTATION_PER_PULSE, 0.0],
    [60.0 * ROTATION_PER_PULSE, 0.0],
    [1.0, 0.0],
];

include!(concat!(env!("OUT_DIR"), "/generated_commands.rs"));

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut pico = Pico::new(env!("CONFIG_FILE"))?;
    
    if pico.sim_mode {
        println!("Running in simulation mode.");
    }

    match pico.get_firmware_version() {
        Ok((major, minor, patch)) => println!("Firmware: {}.{}.{}", major, minor, patch),
        Err(e) => println!("Error: {}", e),
    }

    // --- Dynamic Logging Session Capture Example ---
    let target_motor = 0;
    let log_mask = 5; // Combine metrics (Bits 0, 1, and 2)
    let sampling_rate_ms = 1;

    // Clear stale state info safely before a new motion loop begins
    pico.clear_motor_event(target_motor);
    
    pico.start_logger(0, sampling_rate_ms)?;
    thread::sleep(Duration::from_secs(1));

    pico.move_motor_abs_pos_trapezoid(target_motor, 0.0, 1000.0, 1000.0)?;
    pico.wait_move_done(0, Duration::from_secs(20))?;

    thread::sleep(Duration::from_secs(1));
    pico.stop_logger(0)?;

    let mut collected_logs: Vec<LogEntry> = Vec::new();

    while let Ok(entry) = pico.log_rx.try_recv() {
        collected_logs.push(entry);
    }

    let folder_tag: &str = "TrapezoidRun";
    let now = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();
    let tag = format!("{}", now);
    
    let log_dir = if folder_tag.is_empty() {
        format!("LOG/{}", tag)
    } else {
        format!("LOG/{}", folder_tag)
    };

    fs::create_dir_all(&log_dir)?;
    let file_path = format!("{}/log_{}.csv", log_dir, tag);

    let mut selected_data = Vec::new();
    let mut columns = vec!["Timestamp(ms)".to_string()];

    for bit in 0..N_ENUM {
        let flag = 1 << bit;
        if (log_mask & flag) != 0 {
            selected_data.push(bit);
            columns.push(LOG_MASK_NAMES[bit].to_string());
        }
    }

    let file = fs::File::create(&file_path)?;
    let mut writer = csv::Writer::from_writer(file);
    writer.write_record(&columns)?;

    for data_line in collected_logs {
        let mut row = vec![data_line.dt.to_string()];

        for &idx in &selected_data {
            if idx < data_line.values.len() {
                let scale = SCALE_OFFSET_MOTOR[idx][0];
                let offset = SCALE_OFFSET_MOTOR[idx][1]; 
                let processed_value = (data_line.values[idx] as f64 * scale) + offset;
                row.push(format!("{:.4}", processed_value));
            }
        }
        writer.write_record(&row)?;
    }

    writer.flush()?;
    println!("INFO: Firmware Logger has been saved on: {}", file_path);    

    match pico.get_motor_pos(0) {
        Ok(value) => println!("Final Motor Pos = {}", value),
        Err(e) => println!("Error: {}", e),
    }

    Ok(())
}