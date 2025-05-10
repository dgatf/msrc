#ifndef SMART_ESC_H
#define SMART_ESC_H

#include "common.h"

typedef struct smart_esc_parameters_t {
    bool calc_consumption;
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *voltage_bec,
        *current_bec;                                    // esc
    float *temperature_bat, *current_bat, *consumption;  // bat realtime
    float *cell[18];                                     // cells
    uint8_t *cells;                                      // bat id
    uint16_t *cycles;                                    // bat id
} smart_esc_parameters_t;

extern context_t context;

void smart_esc_task(void *parameters);

#endif