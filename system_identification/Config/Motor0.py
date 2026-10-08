"""Read the shared motor values directly from config/motor_config.toml."""

# Python (in system_identification):
#   from Config import Motor0 as motor_config
#   dt = motor_config.DT_S

from pathlib import Path
import tomllib

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
IDENTIFICATION_CSV = CONFIG_DIR / "system_identification.csv"
with (CONFIG_DIR / "motor_config.toml").open("rb") as _file:
    _settings = tomllib.load(_file)

MOTOR_ID = _settings["MOTOR_ID"]

# Mechanical properties
GEAR_RATIO = _settings["GEAR_RATIO"]
ENCODER_PPR = _settings["ENCODER_PPR"]
ROTATION_PER_PULSE = _settings["ROTATION_PER_PULSE"]
PULSE_PER_ROTATION = _settings["PULSE_PER_ROTATION"]
MAX_SPEED_PPS = _settings["MAX_SPEED_PPS"]
MAX_SPEED_RPM = _settings["MAX_SPEED_RPM"]

# Electronic properties
SYSTEM_FREQ_HZ = _settings["SYSTEM_FREQ_HZ"]
PWM_FREQ_HZ = _settings["PWM_FREQ_HZ"]
MAX_PWM_TICKS = _settings["MAX_PWM_TICKS"]

# Sampling and linear-model properties
FREQUENCY_SAMPLING_HZ = _settings["FREQUENCY_SAMPLING_HZ"]
DT_S = _settings["DT_S"]
K_POSITIVE = _settings["K_POSITIVE"]
K_NEGATIVE = _settings["K_NEGATIVE"]
TAU_S = _settings["TAU_S"]
D_S = _settings["D_S"]
D_STEPS = _settings["D_STEPS"]

# Compatibility with existing Python callers.
motor_id = MOTOR_ID
DELAY_TIME_S = D_S
DELAY_STEPS = D_STEPS
