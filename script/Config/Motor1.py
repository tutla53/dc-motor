# Motor Config

motor_id = 1

# Mechanical Properties
GEAR_RATIO              = 4.4
ENCODER_PPR             = 11
ROTATION_PER_PULSE      = 1/(GEAR_RATIO*ENCODER_PPR)
MAX_SPEED_RPM           = 1200  # Physical Limit
MAX_SPEED_RPM_CONTROL   = 1400  # Control Calculation Limit

# Electronic Properties
SYSTEM_FREQ_HZ          = 133_000_000
PWM_FREQ_HZ             = 25_000
MAX_PWM_TICKS           = (SYSTEM_FREQ_HZ / PWM_FREQ_HZ) - 1; # 25kHz Period = (125_000_000 (Pico clock)/25_000(Frequency)) -1

# System Properties Based on the System Identification
FREQUENCY_SAMPLING_HZ   = 200
DT_S                    = 1/FREQUENCY_SAMPLING_HZ

## Linear Model Properties
K                       = 0.19686   # (pulse per seconds)/PWM_TICKS
TAU_S                   = 0.02911   # seconds
DELAY_TIME_S            = 0.01867   # seconds
DELAY_STEPS             = int(DELAY_TIME_S / DT_S)

## Nonlinear Model Properties
PWM_LIST = [
    -5320, -5054, -4788, -4522, -4256, -3990, -3724, -3458, -3192,
    -2926, -2660, -2394, -2128, -1862, -1596, -1330, -1064, 
    0,
    1062, 1328, 1594, 1860, 2126, 2392, 2658, 2924, 3190, 
    3456, 3722, 3988, 4254, 4520, 4786, 5052, 5318
]

K_LIST = [
    0.21535, 0.22641, 0.23769, 0.23778, 0.22309, 0.21372, 0.20766, 0.20161, 0.19809, # Negative Direction
    0.19563, 0.19336, 0.18926, 0.18510, 0.17802, 0.16551, 0.13973, 0.07920,
    0.0,
    0.07791, 0.14704, 0.17443, 0.18534, 0.19201, 0.19645, 0.19990, 0.20213, 0.20479, # Positive Direction
    0.20847, 0.21517, 0.21718, 0.23149, 0.24240, 0.24560, 0.23367, 0.22216, 
]