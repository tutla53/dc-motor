
use super::*;

pub struct Motor {
    pico: Arc<Mutex<Pico>>,
    pub motor_id: u8,
}

#[allow(unused)]
impl Motor {
    pub fn new(pico: Arc<Mutex<Pico>>, motor_id: u8) -> Self {
        Self {
            pico,
            motor_id,
        }
    }
    
    pub fn stop_motor(&self) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.stop_motor(self.motor_id);
        }
    }

    pub fn move_motor_speed(&self, speed: Speed) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.move_motor_speed(self.motor_id, speed.cps);
        }
    }

    pub fn move_motor_pos_step(&self, target: Position,) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.move_motor_abs_pos(self.motor_id, target.count);
        }
    }

    pub fn move_motor_pos_trapezoid(&self, target: Position, speed: Speed, acc: Acceleration) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.move_motor_abs_pos_trapezoid(self.motor_id, target.count, speed.cps, acc.cps_square);
        }
    }

    pub fn move_motor_open_loop(&self, pwm: i32) {
        if let Ok(mut pico) = self.pico.lock() {
            let _ = pico.move_motor_open_loop(self.motor_id, pwm);
        }
    }    

    pub fn get_motor_pos(&self) -> Result<Position, String> {
        if let Ok(mut pico) = self.pico.lock() {            
            if let Ok(count) = pico.get_motor_pos(self.motor_id) {
                return Ok(Position::from_count(count));
            }
        }
        return Err("Pico returned a runtime error flag for this command".to_string());
    }

    pub fn get_motor_speed(&self) -> Result<Speed, String> {
        if let Ok(mut pico) = self.pico.lock() {            
            if let Ok(cps) = pico.get_motor_speed(self.motor_id) {
                return Ok(Speed::from_cps(cps));
            }
        }
        return Err("Pico returned a runtime error flag for this command".to_string());
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