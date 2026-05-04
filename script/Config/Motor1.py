# Motor Config

motor_id = 1

# Mechanical Properties
GEAR_RATIO              = 4.4
ENCODER_PPR             = 11
ROTATION_PER_PULSE      = 1/(GEAR_RATIO*ENCODER_PPR)
MAX_SPEED_RPM           = 1200

# Electronic Properties
SYSTEM_FREQ_HZ          = 133_000_000
PWM_FREQ_HZ             = 25_000
MAX_PWM_TICKS           = (SYSTEM_FREQ_HZ / PWM_FREQ_HZ) - 1; # 25kHz Period = (125_000_000 (Pico clock)/25_000(Frequency)) -1

# System Properties Based on the System Identification @ 5 ms sampling period
K                       = 0.241583064142491
TAU_S                   = 0.02879561246727829
FREQUENCY_SAMPLING_HZ   = 200
DELAY_TIME_S            = 0.018659895138243635

DT_S                    = 1/FREQUENCY_SAMPLING_HZ
DELAY_STEPS             = int(DELAY_TIME_S / DT_S)

'''
    First Order System with Delay
    Transfer Function
                K * e^(-L * s)
        G(s) = ----------------
                τ * s + 1
    
    Where:
        K   = Gain
        τ   = Time Constant (seconds)
        L   = Delay Time (seconds)
    
    Discrete Form: 
            y[k] = a·y[k-1] + b·u[k-d-1]
'''