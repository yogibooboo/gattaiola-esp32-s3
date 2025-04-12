#ifndef _ANALOGREADFAST_H_
#define _ANALOGREADFAST_H_

#include <stdint.h>
#include <stdbool.h>
#include <hal/adc_hal.h>
#include <esp32-hal-adc.h>

//#define FADC_CAL_USE
#define FADC_RESOLUTION           12
#define FADC_ATTEN          ADC_11db

#ifdef FADC_CAL_USE
#define FADC_CAL_SIZE              6
#define FADC_CAL_RESOLUTION       12
#define FADC_SHIFT (FADC_CAL_RESOLUTION - FADC_RESOLUTION)
#endif

void fadcInit(uint8_t pins, ...);

#ifdef FADC_CAL_USE
uint16_t fadcApply(uint32_t v);
#endif

static inline void __attribute__((always_inline)) fadcStart(uint8_t channel) {
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << channel);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
}

static inline bool __attribute__((always_inline)) fadcBusy() {
    return !(bool)SENS.sar_meas1_ctrl2.meas1_done_sar;
}

static inline uint16_t __attribute__((always_inline)) fadcResult() {
    return HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas1_ctrl2, meas1_data_sar);
}

static inline uint16_t __attribute__((always_inline)) analogReadFast(uint8_t channel) {
    fadcStart(channel);
    while(fadcBusy());
    return fadcResult();
}

#ifdef FADC_CAL_USE
static inline uint16_t __attribute__((always_inline)) analogReadMilliVoltsFast(uint8_t channel) {
    return fadcApply(analogReadFast(channel) << FADC_SHIFT);
}
#endif

#endif