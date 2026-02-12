/*
    Resource Allocation Module
*/

use super::*;

assign_resources! {
    motor_0: Motor0Resources {
        Motor_PWM_CW_PIN: PIN_15,
        Motor_PWM_CCW_PIN: PIN_14,
        Encoder_PIN_A: PIN_6,
        Encoder_PIN_B: PIN_7,
        SLICE: PWM_SLICE7,
    },

    // Still Dummy
    motor_1: Motor1Resources {
        Motor_PWM_CW_PIN: PIN_3,
        Motor_PWM_CCW_PIN: PIN_2,
        Encoder_PIN_A: PIN_4,
        Encoder_PIN_B: PIN_5,
        SLICE: PWM_SLICE1,
    },
}

bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<peripherals::PIO1>;
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});
