// Nonlinear Properties
pub const IDENTIFICATION_CSV: &str = include_str!("system_identification.csv");

#[derive(Debug, Clone, Copy, serde::Deserialize)]
pub struct IdentificationPoint {
    #[serde(rename = "PWM")]
    pub pwm: f64,

    #[serde(rename = "K")]
    pub gain: f64,

    #[serde(rename = "tau")]
    pub tau_s: f64,
}

pub fn load_identification() -> Result<Box<[IdentificationPoint]>, Box<dyn std::error::Error>> {
    let mut reader = csv::Reader::from_reader(IDENTIFICATION_CSV.as_bytes());

    let points = reader
        .deserialize()
        .collect::<Result<Vec<IdentificationPoint>, csv::Error>>()?;

    if points.len() < 2 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "system identification requires at least two rows",
        )
        .into());
    }

    if points.iter().any(|point| {
        !point.pwm.is_finite()
            || !point.gain.is_finite()
            || !point.tau_s.is_finite()
            || point.tau_s <= 0.0
    }) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "identification values must be finite and tau must be positive",
        )
        .into());
    }

    if points.windows(2).any(|pair| pair[0].pwm >= pair[1].pwm) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "PWM values must be strictly increasing",
        )
        .into());
    }

    Ok(points.into_boxed_slice())
}
