use super::*;

struct ActiveFlagConfig {
    data_idx: usize,
    scale: f64,
    offset: f64,
}

pub struct Logger {
    pico: Arc<Mutex<Pico>>,
    mask: LogMask,
    collected_logs: Arc<Mutex<Vec<LogEntry>>>,
    motor_id: u8,
    is_logging_start: Arc<AtomicBool>,
}

impl Logger {
    pub fn new(pico: Arc<Mutex<Pico>>, log_rx: Receiver<LogEntry>, motor_id: u8) -> Self {
        let collected_logs = Arc::new(Mutex::new(Vec::new()));
        let is_logging_start = Arc::new(AtomicBool::new(false));

        let logs_clone = Arc::clone(&collected_logs);
        let active_clone = Arc::clone(&is_logging_start);

        thread::spawn(move || {
            let mut local_buffer = Vec::with_capacity(100);
            let mut last_flush = std::time::Instant::now();

            while let Ok(entry) = log_rx.recv() {
                if active_clone.load(Ordering::Relaxed) {
                    local_buffer.push(entry);

                    if last_flush.elapsed().as_millis() >= 50 || local_buffer.len() >= 100 {
                        if let Ok(mut logs) = logs_clone.lock() {
                            logs.append(&mut local_buffer);
                        }
                        last_flush = std::time::Instant::now();
                    }
                } else {
                    if !local_buffer.is_empty() {
                        local_buffer.clear();
                    }
                }
            }
        });

        Self {
            pico,
            mask: LogMask::NoLog,
            collected_logs,
            motor_id,
            is_logging_start,
        }
    }

    pub fn start(&mut self, mask: LogMask, sampling_rate_ms: u64) {
        if self.is_logging_start.load(Ordering::Relaxed) {
            println!(
                "  [WARN] - {}",
                "FW Logger is already started".bright_red().bold()
            );
            return;
        }

        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.stop_logger(self.motor_id);
        }

        if let Ok(mut logs) = self.collected_logs.lock() {
            logs.clear();
        }

        self.mask = mask;

        self.is_logging_start.store(true, Ordering::Relaxed);

        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.start_logger(self.motor_id, sampling_rate_ms);
        }
    }

    pub fn stop(&mut self) {
        if !self.is_logging_start.load(Ordering::Relaxed) {
            println!(
                "  [WARN] - {}",
                "FW Logger has not been started".bright_red().bold()
            );
            return;
        }

        self.is_logging_start.store(false, Ordering::Relaxed);

        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.stop_logger(self.motor_id);
        }

        let active_mask = self.mask;

        let collected_logs = match self.collected_logs.lock() {
            Ok(mut logs) => std::mem::take(&mut *logs),
            Err(_) => Vec::new(),
        };

        if collected_logs.is_empty() {
            println!("{}", "No Data Collected".bright_red().bold());
            return;
        }

        let folder_tag: &str = "TrapezoidRun";
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let tag = format!("{}", now);

        let log_dir = if folder_tag.is_empty() {
            format!("LOG/{}", tag)
        } else {
            format!("LOG/{}", folder_tag)
        };

        let _ = fs::create_dir_all(&log_dir);
        let file_path = format!("{}/log_{}.csv", log_dir, tag);

        let columns_name = active_mask.get_active_names();
        let active_configs: Vec<ActiveFlagConfig> = active_mask
            .iter()
            .map(|flag| {
                let idx = flag.bits().trailing_zeros() as usize;
                let scale_offset = flag.get_scale_offset();
                ActiveFlagConfig {
                    data_idx: idx,
                    scale: scale_offset.0,
                    offset: scale_offset.1,
                }
            })
            .collect();

        let file = fs::File::create(&file_path).unwrap();
        let mut writer = csv::Writer::from_writer(file);
        let _ = writer.write_record(&columns_name);

        for data_line in collected_logs {
            let _ = writer.write_field(data_line.dt.to_string());

            for config in &active_configs {
                let raw_val = data_line.values[config.data_idx] as f64;
                let processed_value = (raw_val * config.scale) + config.offset;
                
                let _ = writer.write_field(format!("{:.4}", processed_value));
            }

            let _ = writer.write_record(None::<&[u8]>);
        }

        let _ = writer.flush();
        println!(
            "  [INFO] {} {}",
            "- Firmware Logger has been saved on:".bright_yellow(),
            file_path
        );
    }
}
