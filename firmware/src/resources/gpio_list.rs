//  Resource Allocation Module

use {
    assign_resources::assign_resources,
    embassy_rp::{
        bind_interrupts,
        peripherals,
        pio::InterruptHandler as PioInterruptHandler,
        usb::InterruptHandler as UsbInterruptHandler,
    },
};

assign_resources! {
    motor_resources: MotorResources {
        Motor0_PWM_CW_PIN: PIN_15,
        Motor0_PWM_CCW_PIN: PIN_14,
    },

    encoder_resources: EncoderResources {
        Encoder0_PIN_A: PIN_6,
        Encoder0_PIN_B: PIN_7,
    },
}

bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});
