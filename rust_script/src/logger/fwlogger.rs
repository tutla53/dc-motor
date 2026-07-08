
#![allow(unused)]

use super::*;

pub struct Logger {
    pico: Arc<Mutex<Pico>>,
    log_rx: Option<Receiver<LogEntry>>, 
    mask: i32,
    collected_logs: Arc<Mutex<Vec<LogEntry>>>,
    motor_id: u8,
    is_logging_start: bool, 
}

impl Logger {
    pub fn new(pico: Arc<Mutex<Pico>>, log_rx: Receiver<LogEntry>, motor_id:u8) -> Self {
        Self { 
            pico,
            log_rx: Some(log_rx), 
            mask: 0,
            collected_logs: Arc::new(Mutex::new(Vec::new())),
            motor_id,
            is_logging_start: false,
        }
    }

    pub fn start(&mut self, mask: i32, sampling_rate_ms: u64) {

        if self.is_logging_start {
            println!("  [WARN] - {}", "FW Logger is already started".bright_red().bold());
        }
        else {
            self.is_logging_start = true;
            self.mask = mask;

            if let Some(rx) = self.log_rx.take() {
                let logs_clone = Arc::clone(&self.collected_logs);
                thread::spawn(move || {
                    let mut local_buffer = Vec::with_capacity(100);
                    let mut last_flush = std::time::Instant::now();

                    while let Ok(entry) = rx.recv() {
                        local_buffer.push(entry);

                        if last_flush.elapsed().as_millis() >= 50 || local_buffer.len() >= 100 {
                            if let Ok(mut logs) = logs_clone.lock() {
                                logs.append(&mut local_buffer);
                            }
                            last_flush = std::time::Instant::now();
                        }
                    }

                    if !local_buffer.is_empty() {
                        if let Ok(mut logs) = logs_clone.lock() {
                            logs.append(&mut local_buffer);
                        }
                    }
                });
            }

            if let Ok(mut pico) = self.pico.lock() {
                let _ = pico.start_logger(self.motor_id, sampling_rate_ms);
            }
        }
    }

    pub fn stop(&mut self) {
        if !self.is_logging_start {
            println!("  [WARN] - {}", "FW Logger has not been started".bright_red().bold());
        }
        else {
            self.is_logging_start = false;

            if let Ok(mut pico) = self.pico.lock() {
                let _ = pico.stop_logger(self.motor_id);
            }

            let collected_logs = match self.collected_logs.lock() {
                Ok(mut logs) => std::mem::take(&mut *logs),
                Err(_) => Vec::new(),
            };

            if collected_logs.is_empty() {
                println!("{}", "No Data Collected".bright_red().bold());
                return
            }
            let folder_tag: &str = "TrapezoidRun";
            let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
            let tag = format!("{}", now);
            
            let log_dir = if folder_tag.is_empty() {
                format!("LOG/{}", tag)
            } else {
                format!("LOG/{}", folder_tag)
            };

            let _ = fs::create_dir_all(&log_dir);
            let file_path = format!("{}/log_{}.csv", log_dir, tag);

            let mut selected_data = Vec::new();
            let mut columns = vec!["Timestamp(ms)".to_string()];

            for bit in 0..logger_config::N_ENUM {
                let flag = 1 << bit;
                if (self.mask & flag) != 0 {
                    selected_data.push(bit);
                    columns.push(logger_config::LOG_MASK_NAMES[bit].to_string());
                }
            }

            let file = fs::File::create(&file_path).unwrap();
            let mut writer = csv::Writer::from_writer(file);
            let _ = writer.write_record(&columns);

            for data_line in collected_logs {
                let mut row = vec![data_line.dt.to_string()];

                for &idx in &selected_data {
                    if idx < data_line.values.len() {
                        let scale = logger_config::SCALE_OFFSET_MOTOR[idx][0];
                        let offset = logger_config::SCALE_OFFSET_MOTOR[idx][1]; 
                        let processed_value = (data_line.values[idx] as f64 * scale) + offset;
                        row.push(format!("{:.4}", processed_value));
                    }
                }
                let _ = writer.write_record(&row);
            }

            let _ = writer.flush();
            println!("  [INFO] {} {}", "- Firmware Logger has been saved on:".bright_yellow(), file_path);    
        }
    }
}