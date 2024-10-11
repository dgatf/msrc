#ifndef ESC_HW5_H
#define ESC_HW5_H

#include "common.h"

typedef struct esc_hw5_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *temperature_motor, *voltage_bec, *current_bec,
        *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_hw5_parameters_t;

extern context_t context;

void esc_hw5_task(void *parameters);

#endif