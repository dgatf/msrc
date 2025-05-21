#ifndef ESC_ZTW_H
#define ESC_ZTW_H

#include "common.h"

typedef struct esc_ztw_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temp_esc, *temp_motor, *bec_voltage, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_ztw_parameters_t;

extern context_t context;

void esc_ztw_task(void *parameters);

#endif