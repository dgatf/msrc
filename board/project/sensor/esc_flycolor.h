#ifndef ESC_FLYCOLOR_H
#define ESC_FLYCOLOR_H

#include "common.h"

typedef struct esc_flycolor_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature, *cell_voltage, *consumption;
    uint8_t *cell_count;
    volatile bool request_telemetry;
} esc_flycolor_parameters_t;

extern context_t context;

void esc_flycolor_task(void *parameters);

#endif