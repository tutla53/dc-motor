"""Read the shared motor values directly from config/motor_config.toml."""

from pathlib import Path
import tomllib

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
IDENTIFICATION_CSV = CONFIG_DIR / "system_identification.csv"
with (CONFIG_DIR / "motor_config.toml").open("rb") as _file:
    _settings = tomllib.load(_file)

motor_id = _settings["motor"]["motor_id"]

# Mechanical properties
GEAR_RATIO = _settings["motor"]["gear_ratio"]
ENCODER_PPR = _settings["motor"]["encoder_ppr"]
ROTATION_PER_PULSE = _settings["motor"]["rotation_per_pulse"]
PULSE_PER_ROTATION = _settings["motor"]["pulse_per_rotation"]
MAX_SPEED_PPS = _settings["motor"]["max_speed_pps"]
MAX_SPEED_RPM = _settings["motor"]["max_speed_rpm"]

# Electronic properties
SYSTEM_FREQ_HZ = _settings["electronics"]["system_freq_hz"]
PWM_FREQ_HZ = _settings["electronics"]["pwm_freq_hz"]
MAX_PWM_TICKS = _settings["electronics"]["max_pwm_ticks"]

# Sampling and linear-model properties
FREQUENCY_SAMPLING_HZ = _settings["sampling"]["frequency_sampling_hz"]
DT_S = _settings["sampling"]["dt_s"]
K_POSITIVE = _settings["linear_model"]["k_positive"]
K_NEGATIVE = _settings["linear_model"]["k_negative"]
TAU_S = _settings["linear_model"]["tau_s"]
DELAY_TIME_S = _settings["linear_model"]["delay_time_s"]
DELAY_STEPS = _settings["linear_model"]["delay_steps"]
