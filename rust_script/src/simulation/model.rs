use super::*;

#[derive(Debug, Clone)]
pub struct OverlaySeries {
    pub label: String,
    pub points: Vec<(f32, f32)>,
    pub color: RGBColor,
}

#[allow(clippy::enum_variant_names)]
pub enum SimMode<'a> {
    OpenLoop(f64),
    SpeedClosedLoop(f64, u32, &'a PIDConfig),
    PositionClosedLoop(f64, u32, &'a PIDConfig, &'a PIDConfig),
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum ModelKind {
    Linear,
    Nonlinear,
}

/*
First Order System with Delay
    Input:
        pwm (ticks)
    Output:
        speed (pulse per seconds)

    Transfer Function
                K * e^(-L * s)
        G(s) = ----------------
                τ * s + 1

    Where:
        K   = Gain          ((pulse per seconds)/ ticks)
        τ   = Time Constant (seconds)
        L   = Delay Time    (seconds)

    Discrete Form with sampling time dt:
        y[k] = a·y[k-1] + b·u[k-d-1]

        with:
            a = e^(-dt / τ)
            b = K * (1-a)
            d = time_delay
*/

pub struct MotorSimulation {
    model_kind: ModelKind,
    identification: Box<[nonlinear::IdentificationPoint]>,
    alpha: f64,
    beta_positive: f64,
    beta_negative: f64,
    speed_control: PIDController<I16F16>,
    position_control: PIDController<I32F32>,
}

impl MotorSimulation {
    /* ---------- Initialization ---------- */
    pub fn new(model_kind: ModelKind) -> Result<Self, Box<dyn std::error::Error>> {
        let alpha = (-motor_config::DT_S / motor_config::TAU_S).exp();

        let identification = match model_kind {
            ModelKind::Linear => Vec::new().into_boxed_slice(),
            ModelKind::Nonlinear => nonlinear::load_identification()?,
        };

        let speed_control: PIDController<I16F16> = PIDController::new(
            motor_config::DEFAULT_PID_SPEED_CONFIG,
            motor_config::MAX_PWM_TICKS,
        )?;
        let position_control: PIDController<I32F32> = PIDController::new(
            motor_config::DEFAULT_PID_POS_CONFIG,
            motor_config::MAX_SPEED_PPS,
        )?;

        Ok(Self {
            model_kind,
            alpha,
            identification,
            beta_positive: motor_config::K_POSITIVE * (1.0 - alpha),
            beta_negative: motor_config::K_NEGATIVE * (1.0 - alpha),
            speed_control,
            position_control,
        })
    }

    fn interpolate_gain(&self, input_pwm: f64) -> f64 {
        let points = &self.identification;

        if input_pwm <= points[0].pwm {
            return points[0].gain;
        }

        if input_pwm >= points[points.len() - 1].pwm {
            return points[points.len() - 1].gain;
        }

        let upper = points.partition_point(|point| point.pwm < input_pwm);
        let lower = upper - 1;

        let p0 = points[lower];
        let p1 = points[upper];
        let weight = (input_pwm - p0.pwm) / (p1.pwm - p0.pwm);

        p0.gain + weight * (p1.gain - p0.gain)
    }

    fn beta(&self, input_pwm: f64) -> f64 {
        match self.model_kind {
            ModelKind::Linear => {
                if input_pwm >= 0.0 {
                    self.beta_positive
                } else {
                    self.beta_negative
                }
            }
            ModelKind::Nonlinear => self.interpolate_gain(input_pwm) * (1.0 - self.alpha),
        }
    }

    /* ---------- Wrapper ---------- */
    pub fn simulate_open_loop(
        log: &CsvProcessing,
        model_kind: ModelKind,
    ) -> Result<OverlaySeries, Box<dyn std::error::Error>> {
        Self::new(model_kind)?.core(log, SimMode::OpenLoop(0.0))
    }

    pub fn simulate_speed_control(
        log: &CsvProcessing,
        model_kind: ModelKind,
        max_speed_pps: u32,
        pid_config: &PIDConfig,
    ) -> Result<OverlaySeries, Box<dyn std::error::Error>> {
        Self::new(model_kind)?.core(
            log,
            SimMode::SpeedClosedLoop(0.0, max_speed_pps, pid_config),
        )
    }

    pub fn simulate_position_control(
        log: &CsvProcessing,
        model_kind: ModelKind,
        initial_pos: i32,
        max_speed_pps: u32,
        pid_speed_config: &PIDConfig,
        pid_pos_config: &PIDConfig,
    ) -> Result<OverlaySeries, Box<dyn std::error::Error>> {
        Self::new(model_kind)?.core(
            log,
            SimMode::PositionClosedLoop(
                initial_pos as f64,
                max_speed_pps,
                pid_speed_config,
                pid_pos_config,
            ),
        )
    }

    /* ---------- Mathematical Model ---------- */
    fn open_loop(&self, initial_condition: f64, u: Vec<f64>) -> Vec<f64> {
        let mut y = vec![initial_condition; u.len()];
        let d = motor_config::L_STEPS as usize;

        /* ---------- Difference Equation ---------- */
        for k in 0..u.len() {
            if (k as i32 - d as i32 - 1) < 0 {
                continue;
            }

            y[k] = self.alpha * y[k - 1] + self.beta(u[k - d - 1]) * u[k - d - 1];
        }

        y
    }

    fn speed_control(
        &mut self,
        initial_condition: f64,
        set_point: Vec<f64>,
        max_speed_pps: u32,
        pid_config: &PIDConfig,
    ) -> Result<Vec<f64>, Box<dyn std::error::Error>> {
        let mut y = vec![initial_condition; set_point.len()];
        let mut u = vec![0.0; set_point.len()];
        let d = motor_config::L_STEPS as usize;

        self.speed_control.update_pid_param(*pid_config)?;
        self.speed_control
            .update_max_output(motor_config::MAX_PWM_TICKS)?;

        /* ---------- Difference Equation ---------- */
        for k in 0..set_point.len() {
            if (k as i32 - d as i32 - 1) < 0 {
                continue;
            }

            u[k] = self.speed_control.compute(
                (set_point[k] as i32).clamp(-(max_speed_pps as i32), max_speed_pps as i32),
                I16F16::from_num(y[k - 1]),
            ) as f64;

            y[k] = self.alpha * y[k - 1] + self.beta(u[k - d - 1]) * u[k - d - 1];
        }

        Ok(y)
    }

    fn position_control(
        &mut self,
        initial_condition: f64,
        set_point: Vec<f64>,
        max_speed_pps: u32,
        pid_speed_config: &PIDConfig,
        pid_pos_config: &PIDConfig,
    ) -> Result<Vec<f64>, Box<dyn std::error::Error>> {
        let mut x = vec![initial_condition; set_point.len()]; // Motor Position
        let mut y = vec![0.0; set_point.len()]; // Motor Speed Output
        let mut u = vec![0.0; set_point.len()]; // PWM Input
        let d = motor_config::L_STEPS as usize;

        self.speed_control.update_pid_param(*pid_speed_config)?;
        self.speed_control
            .update_max_output(motor_config::MAX_PWM_TICKS)?;

        self.position_control.update_pid_param(*pid_pos_config)?;
        self.position_control.update_max_output(max_speed_pps)?;

        /* ---------- Difference Equation ---------- */
        for k in 0..set_point.len() {
            if (k as i32 - d as i32 - 1) < 0 {
                continue;
            }

            let target_speed = self
                .position_control
                .compute(set_point[k] as i32, I32F32::from_num(x[k - 1]));

            u[k] = self
                .speed_control
                .compute(target_speed, I16F16::from_num(y[k - 1])) as f64;

            y[k] = self.alpha * y[k - 1] + self.beta(u[k - d - 1]) * u[k - d - 1]; // Updating Motor Speed
            x[k] = x[k - 1] + ((y[k - 1] + y[k]) / 2.0) * motor_config::DT_S; // Update Position
        }

        Ok(x)
    }

    /* ---------- Mode Selection ---------- */
    fn core(
        &mut self,
        log: &CsvProcessing,
        mode: SimMode,
    ) -> Result<OverlaySeries, Box<dyn std::error::Error>> {
        let (commanded_header, legend, input_converter, output_converter) = match mode {
            SimMode::OpenLoop(_) => {
                (
                    "Commanded_PWM",
                    "Open Loop Simulation",
                    1.0,                                     // pwm to pwm
                    motor_config::ROTATION_PER_COUNT * 60.0, // pps to rpm
                )
            }
            SimMode::SpeedClosedLoop(_, _, _) => {
                (
                    "Commanded_Speed(RPM)",
                    "Motor Speed Simulation",
                    1.0 / (motor_config::ROTATION_PER_COUNT * 60.0), // rpm to pps
                    motor_config::ROTATION_PER_COUNT * 60.0,         // pps to rpm
                )
            }
            SimMode::PositionClosedLoop(_, _, _, _) => {
                (
                    "Commanded_Position(rotation)",
                    "Motor Position Simulation",
                    1.0 / motor_config::ROTATION_PER_COUNT, // rotation to pulse
                    motor_config::ROTATION_PER_COUNT,       // pulse to rotation
                )
            }
        };

        let pwm_column = log
            .header_names
            .iter()
            .position(|name| name == commanded_header)
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "Commanded_PWM column is missing"))?;

        /* ---------- Set Point ---------- */
        let u: Vec<f64> = log
            .data
            .iter()
            .map(|row| (row[pwm_column] as f64) * input_converter)
            .collect();

        /* ---------- Calculate Motor Output ---------- */
        let y = match mode {
            SimMode::OpenLoop(initial_condition) => self.open_loop(initial_condition, u),
            SimMode::SpeedClosedLoop(initial_condition, max_speed_pps, pid_config) => {
                self.speed_control(initial_condition, u, max_speed_pps, pid_config)?
            }
            SimMode::PositionClosedLoop(
                initial_condition,
                max_speed_pps,
                pid_speed_config,
                pid_pos_config,
            ) => self.position_control(
                initial_condition,
                u,
                max_speed_pps,
                pid_speed_config,
                pid_pos_config,
            )?,
        };

        /* ---------- Convert Simulation Unit ---------- */
        let points = log
            .data
            .iter()
            .zip(y)
            .map(|(row, current_data)| {
                let time_s = row[0];
                let output = current_data * output_converter;
                (time_s, output as f32)
            })
            .collect();

        Ok(OverlaySeries {
            label: legend.to_owned(),
            points,
            color: RGBColor(0, 72, 140),
        })
    }
}
