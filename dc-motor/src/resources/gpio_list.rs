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
    network_resources: NetworkResources {
        CYW43_PWR_PIN: PIN_23,
        CYW43_CS_PIN: PIN_25,
        CYW43_PIO_CH: PIO0,
        CYW43_SPI_DIO: PIN_24,
        CYW43_SPI_CLK: PIN_29,
        CYW43_DMA_CH: DMA_CH0,
    },

    adc_resources: AdcResources {
        ADC_PERIPHERAL: ADC,
        ADC_TEMP_PIN: ADC_TEMP_SENSOR,
    },
}

bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});
