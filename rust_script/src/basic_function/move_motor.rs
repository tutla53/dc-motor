
use super::*;

pub struct Motor {
    pico: Arc<Mutex<Pico>>,
    motor_id: u8,
}

impl Motor {
    pub fn new(pico: Arc<Mutex<Pico>>, motor_id: u8) -> Self {
        Self {
            pico,
            motor_id,
        }
    }

    pub fn move_motor_trapezoid(&self, target:f32, speed:f32, acc:f32) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.move_motor_abs_pos_trapezoid(self.motor_id, target, speed, acc);
        }
    }

    pub fn clear_motor_event(&self) {
        if let Ok(pico) = self.pico.lock() {
            if let Ok(mut events) = pico.shared_events.lock() {
                events.remove(&self.motor_id);
            }
        }
    }

    pub fn wait_move_done(&self, timeout: Duration) -> Result<u8, String> {
        if let Ok(pico) = self.pico.lock() {
            if pico.sim_mode {
                thread::sleep(Duration::from_millis(500));
                return Ok(0);
            }

            let start = Instant::now();
            loop {
                {
                    let mut events = pico.shared_events.lock().unwrap();
                    if let Some(ev_code) = events.remove(&self.motor_id) {
                        return Ok(ev_code);
                    }
                }
                
                if start.elapsed() > timeout {
                    return Err(format!("Timeout waiting for motor {} to finish moving", self.motor_id));
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
        return Err(format!("Failed to lock the pico"));
    }
}