#include "analog.h"
#include <stdint.h>
#include <stdarg.h>
#include <hal/adc_hal.h>
#include <driver/periph_ctrl.h>

void fadcInit(uint8_t pins, ...) {
    // Initialize ADC using built-ins
    analogReadResolution(FADC_RESOLUTION);
    analogSetAttenuation(FADC_ATTEN);
    
    // Initialize pins using built-ins
    va_list args;
    va_start(args, pins);
    while(pins--) {
        int pin = va_arg(args, int);
        analogSetPinAttenuation(pin, FADC_ATTEN);
        analogRead(pin);
    }
    va_end(args);

    // Enable ADC
    periph_module_enable(PERIPH_SARADC_MODULE);
    // Nota: adc_power_acquire omesso per ora, potrebbe non essere necessario
}