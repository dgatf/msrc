#ifndef ESC_KONTRONIK_H
#define ESC_KONTRONIK_H

#include "common.h"

typedef struct esc_kontronik_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *voltage_bec, *current_bec, *temperature_fet, *temperature_bec, *cell_voltage,
        *consumption;
    uint8_t *cell_count;
} esc_kontronik_parameters_t;

extern context_t context;

void esc_kontronik_task(void *parameters);

#endif