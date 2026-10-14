use super::*;

pub struct SineGenerator {
    amplitude: f64,
    frequency: f64,
    dt: f64,          // Time step in seconds
    phase: f64,       // Current phase in radians [0, 2*PI)
}

impl SineGenerator {
    pub fn new(amplitude: f64, frequency: f64, dt_ms: f64) -> Self {
        Self {
            amplitude,
            frequency,
            dt: dt_ms / 1000.0, // Convert ms to seconds
            phase: 0.0,
        }
    }

    /// Generates the next discrete sample and advances the phase
    pub fn next_sample(&mut self) -> f64 {
        let sample = self.amplitude * self.phase.sin();

        // Advance phase by angular increment: 2 * pi * f * dt
        let delta_phase = 2.0 * PI * self.frequency * self.dt;
        self.phase += delta_phase;

        // Wrap phase to keep it within [0, 2*PI) and prevent precision drift
        if self.phase >= 2.0 * PI {
            self.phase %= 2.0 * PI;
        }

        sample
    }
}