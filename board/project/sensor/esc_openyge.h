#ifndef ESC_OPENYGE_H
#define ESC_OPENYGE_H

#include "common.h"

typedef struct esc_openyge_parameters_t {
    float rpm_multiplier;
    bool pwm_out;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *cell_voltage, *consumption;
    float *voltage_bec, *current_bec, *throttle, *pwm_percent;
    uint8_t *cell_count;
} esc_openyge_parameters_t;

extern context_t context;

void esc_openyge_task(void *parameters);

#endif