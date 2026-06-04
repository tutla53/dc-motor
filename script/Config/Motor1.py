import pandas as pd
from base_url import *

# Motor Config

motor_id = 1

# Mechanical Properties
GEAR_RATIO              = 4.4
ENCODER_PPR             = 11
ROTATION_PER_PULSE      = 1/(GEAR_RATIO*ENCODER_PPR)
MAX_SPEED_PPS           = 887
MAX_SPEED_RPM           = MAX_SPEED_PPS * ROTATION_PER_PULSE * 60

# Electronic Properties
SYSTEM_FREQ_HZ          = 133_000_000
PWM_FREQ_HZ             = 25_000
MAX_PWM_TICKS           = (SYSTEM_FREQ_HZ / PWM_FREQ_HZ) - 1; # 25kHz Period = (125_000_000 (Pico clock)/25_000(Frequency)) -1

# System Properties Based on the System Identification
FREQUENCY_SAMPLING_HZ   = 1000
DT_S                    = 1/FREQUENCY_SAMPLING_HZ

## Linear Model Properties
K                       = 0.27   # (pulse per seconds)/PWM_TICKS
TAU_S                   = 0.0263   # seconds
DELAY_TIME_S            = 0.014     # seconds
DELAY_STEPS             = int(DELAY_TIME_S / DT_S)

## Nonlinear Model Properties
data        = pd.read_csv(base_url+"/Config/system_identification.csv")
PWM_LIST    = data["PWM"].values
K_LIST      = data["K"].values
TAU_LIST    = data["tau"].values