#ifndef ESC_OMP_M4_H
#define ESC_OMP_M4_H

#include "common.h"

typedef struct esc_omp_m4_parameters_t {
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temp_esc, *temp_motor, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_omp_m4_parameters_t;

extern context_t context;

void esc_omp_m4_task(void *parameters);

#endif