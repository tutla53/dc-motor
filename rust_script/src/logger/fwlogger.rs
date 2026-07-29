use super::*;

/*
    Clear the invalid initial timestamp if the initial
    timestamp is more than the TIME_THRESHOLD_MS
*/
const TIME_THRESHOLD_MS: u32 = 200;

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
            let mut was_active = false;
            let mut is_first_packet_of_session = false;

            while let Ok(entry) = log_rx.recv() {
                if active_clone.load(Ordering::Relaxed) {
                    if !was_active {
                        was_active = true;
                        is_first_packet_of_session = true;
                        local_buffer.clear();
                        while log_rx.try_recv().is_ok() {} // Draining the log
                        continue;
                    }

                    if is_first_packet_of_session {
                        is_first_packet_of_session = false;

                        if entry.dt > TIME_THRESHOLD_MS {
                            continue;
                        }
                    }

                    local_buffer.push(entry);

                    if last_flush.elapsed().as_millis() >= 50 || local_buffer.len() >= 100 {
                        if let Ok(mut logs) = logs_clone.lock() {
                            logs.append(&mut local_buffer);
                        }
                        last_flush = std::time::Instant::now();
                    }
                } else {
                    was_active = false;
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

    pub fn start(
        &mut self,
        mask: LogMask,
        sampling_rate_ms: u64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if self.is_logging_start.load(Ordering::Relaxed) {
            return Err(Box::from("FW Logger has been started"));
        }

        run_with_lock!(self.pico => stop_logger(self.motor_id))??;

        run_with_lock!(self.collected_logs => clear())?;
        self.mask = mask;
        self.is_logging_start.store(true, Ordering::Relaxed);

        run_with_lock!(self.pico => start_logger(self.motor_id, sampling_rate_ms))??;

        Ok(())
    }

    pub fn stop(&mut self) -> Result<String, Box<dyn std::error::Error>> {
        if !self.is_logging_start.load(Ordering::Relaxed) {
            return Err(Box::from("FW Logger has not been started"));
        }

        self.is_logging_start.store(false, Ordering::Relaxed);

        run_with_lock!(self.pico => stop_logger(self.motor_id))??;

        let active_mask = self.mask;

        let collected_logs = match self.collected_logs.lock() {
            Ok(mut logs) => std::mem::take(&mut *logs),
            Err(_) => Vec::new(),
        };

        if collected_logs.is_empty() {
            return Err(Box::from("No Data Collected"));
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

        let mut first_data = false;
        let mut normal_timestamp = false;

        for data_line in collected_logs {
            if !normal_timestamp {
                // check unusual timestamp at the beginning
                if data_line.dt > TIME_THRESHOLD_MS {
                    // skip and print the unusual time untill it's normal again
                    if !first_data {
                        first_data = true;
                        println!(
                            "{} - {} {} ticks",
                            "  [WARN]".bright_yellow().bold(),
                            "Found timestamp error at:".bright_red(),
                            data_line.dt,
                        );
                    }
                    continue;
                }

                normal_timestamp = true;
            }

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

        Ok(log_dir)
    }
}
