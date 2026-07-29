use super::*;

pub struct Motor {
    pico: Arc<Mutex<Pico>>,
    pub motor_id: u8,
}

#[allow(unused)]
impl Motor {
    pub fn new(pico: Arc<Mutex<Pico>>, motor_id: u8) -> Self {
        Self { pico, motor_id }
    }

    pub fn stop_motor(&self) {
        if let Err(e) = run_with_lock!(self.pico => stop_motor(self.motor_id)) {
            println!("{}", e);
        }
    }

    pub fn move_motor_speed(&self, speed: Speed) {
        if let Err(e) = run_with_lock!(self.pico => move_motor_speed(self.motor_id, speed.cps)) {
            println!("{}", e);
        }
    }

    pub fn move_motor_pos_step(&self, target: Position) {
        if let Err(e) = run_with_lock!(self.pico => move_motor_abs_pos(self.motor_id, target.count))
        {
            println!("{}", e);
        }
    }

    pub fn move_motor_pos_trapezoid(&self, target: Position, speed: Speed, acc: Acceleration) {
        if let Err(e) = run_with_lock!(self.pico => move_motor_abs_pos_trapezoid(self.motor_id, target.count, speed.cps, acc.cps_square,))
        {
            println!("{}", e);
        }
    }

    pub fn move_motor_open_loop(&self, pwm: i32) {
        if let Err(e) = run_with_lock!(self.pico => move_motor_open_loop(self.motor_id, pwm)) {
            println!("{}", e);
        }
    }

    pub fn get_motor_pos(&self) -> Result<Position, String> {
        if let Ok(count) = run_with_lock!(self.pico => get_motor_pos(self.motor_id)) {
            match count {
                Ok(value) => {
                    return Ok(Position::from_count(value));
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }

        Err("Pico returned a runtime error flag for this command".to_string())
    }

    pub fn get_motor_speed(&self) -> Result<Speed, String> {
        if let Ok(cps) = run_with_lock!(self.pico => get_motor_speed(self.motor_id)) {
            match cps {
                Ok(value) => {
                    return Ok(Speed::from_cps(value));
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }

        Err("Pico returned a runtime error flag for this command".to_string())
    }

    pub fn clear_motor_event(&self) {
        if let Ok(pico) = self.pico.lock()
            && let Ok(mut events) = pico.shared_events.lock()
        {
            events.remove(&self.motor_id);
        }
    }

    pub fn wait_move_done(&self, timeout: Duration) -> Result<u8, String> {
        if let Ok(pico) = self.pico.lock() {
            if pico.is_sim_mode() {
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
                    return Err(format!(
                        "Timeout waiting for motor {} to finish moving",
                        self.motor_id
                    ));
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
        Err("Failed to lock the pico".to_string())
    }
}
