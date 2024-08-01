#ifndef ESC_APD_HV_H
#define ESC_APD_HV_H

#include "common.h"

typedef struct esc_apd_hv_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_apd_hv_parameters_t;

extern context_t context;

void esc_apd_hv_task(void *parameters);

#endif