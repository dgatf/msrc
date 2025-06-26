#ifndef ESC_XDFLY_H
#define ESC_XDFLY_H

#include "common.h"

typedef struct esc_xdfly_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *temp, *voltage, *bec_voltage, *current, *consumption, *cell_voltage;
    uint8_t *cell_count;
} esc_xdfly_parameters_t;

extern context_t context;

void esc_xdfly_task(void *parameters);

#endif