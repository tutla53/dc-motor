
use super::*;

const N_ENUM: usize = 5;

const LOG_MASK_NAMES: [&str; N_ENUM] = [
    "Motor_Position(rotation)",
    "Motor_Speed(RPM)",
    "Commanded_Position(rotation)",
    "Commanded_Speed(RPM)",
    "Commanded_PWM",
];

const GEAR_RATIO: f64           = 4.4;
const ENCODER_PPR: f64          = 11.0;
const ROTATION_PER_PULSE: f64   = 1.0/(GEAR_RATIO*ENCODER_PPR);

const SCALE_OFFSET_MOTOR: [[f64; 2]; N_ENUM] = [
    [ROTATION_PER_PULSE, 0.0],
    [60.0 * ROTATION_PER_PULSE, 0.0],
    [ROTATION_PER_PULSE, 0.0],
    [60.0 * ROTATION_PER_PULSE, 0.0],
    [1.0, 0.0],
];

pub struct Logger {
    pico: Arc<Mutex<Pico>>,
    log_rx: Option<Receiver<LogEntry>>, 
    mask: i32,
    collected_logs: Arc<Mutex<Vec<LogEntry>>>, 
}

impl Logger {
    pub fn new(pico: Arc<Mutex<Pico>>, log_rx: Receiver<LogEntry>) -> Self {
        Self { 
            pico,
            log_rx: Some(log_rx), 
            mask: 0,
            collected_logs: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn start(&mut self, mask: i32, sampling_rate_ms: u64) {
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
            let _ = pico.start_logger(0, sampling_rate_ms);
        }
    }

    pub fn stop(&self) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.stop_logger(0);
        }

        let collected_logs = match self.collected_logs.lock() {
            Ok(mut logs) => std::mem::take(&mut *logs),
            Err(_) => Vec::new(),
        };

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

        for bit in 0..N_ENUM {
            let flag = 1 << bit;
            if (self.mask & flag) != 0 {
                selected_data.push(bit);
                columns.push(LOG_MASK_NAMES[bit].to_string());
            }
        }

        let file = fs::File::create(&file_path).unwrap();
        let mut writer = csv::Writer::from_writer(file);
        let _ = writer.write_record(&columns);

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
            let _ = writer.write_record(&row);
        }

        let _ = writer.flush();
        println!("INFO: Firmware Logger has been saved on: {}", file_path);    
    }
}