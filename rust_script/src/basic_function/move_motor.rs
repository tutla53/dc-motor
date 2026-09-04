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

    pub fn enable(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.set_motor_enable(self.motor_id)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn disable(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.set_motor_disable(self.motor_id)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn is_enabled(&self) -> Result<bool, Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        let result = pico
            .get_motor_enable(self.motor_id)
            .map_err(std::io::Error::other)?;

        match result {
            0 => Ok(false),
            1 => Ok(true),
            value => Err(std::io::Error::other(format!(
                "Invalid motor-enable value from firmware: {value} (expected 0 or 1)"
            ))
            .into()),
        }
    }

    pub fn stop_motor(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.stop_motor(self.motor_id)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn move_motor_speed(&self, speed: Speed) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.move_motor_speed(self.motor_id, speed.cps)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn move_motor_pos_step(&self, target: Position) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.move_motor_abs_pos(self.motor_id, target.count)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn move_motor_pos_trapezoid(
        &self,
        target: Position,
        speed: Speed,
        acc: Acceleration,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.move_motor_abs_pos_trapezoid(self.motor_id, target.count, speed.cps, acc.cps_square)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn move_motor_open_loop(&self, pwm: Pwm) -> Result<(), Box<dyn std::error::Error>> {
        let mut pico = self
            .pico
            .lock()
            .map_err(|_| std::io::Error::other("Pico mutex is poisoned"))?;

        pico.move_motor_open_loop(self.motor_id, pwm.ticks)
            .map_err(std::io::Error::other)?;
        Ok(())
    }

    pub fn get_motor_pos(&self) -> Result<Position, String> {
        if let Ok(count) = try_lock!(self.pico => get_motor_pos(self.motor_id)) {
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
        if let Ok(cps) = try_lock!(self.pico => get_motor_speed(self.motor_id)) {
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

    pub fn get_pid_motor_speed(&self) -> Result<PIDConfig, String> {
        if let Ok(pid_config) = try_lock!(self.pico => get_pid_motor_speed(self.motor_id)) {
            match pid_config {
                Ok((kp, ki, kd, i_limit)) => {
                    return Ok(PIDConfig {
                        kp,
                        ki,
                        kd,
                        i_limit,
                    });
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }

        Err("Pico returned a runtime error flag for this command".to_string())
    }

    pub fn get_pid_motor_pos(&self) -> Result<PIDConfig, String> {
        if let Ok(pid_config) = try_lock!(self.pico => get_pid_motor_pos(self.motor_id)) {
            match pid_config {
                Ok((kp, ki, kd, i_limit)) => {
                    return Ok(PIDConfig {
                        kp,
                        ki,
                        kd,
                        i_limit,
                    });
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }

        Err("Pico returned a runtime error flag for this command".to_string())
    }

    pub fn get_motor_max_speed(&self) -> Result<u32, String> {
        if let Ok(max_speed) = try_lock!(self.pico => get_motor_max_speed(self.motor_id)) {
            match max_speed {
                Ok(value) => {
                    return Ok(value);
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
                        "Timeout waiting for [Motor {}] to finish moving",
                        self.motor_id
                    ));
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
        Err("Failed to lock the pico".to_string())
    }
}
