#ifndef ESC_HW4_H
#define ESC_HW4_H

#include "common.h"

typedef struct esc_hw4_parameters_t {
    float rpm_multiplier;
    bool pwm_out;
    bool init_delay;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float voltage_multiplier, current_multiplier, current_thresold, current_max;
    bool current_is_manual_offset, auto_detect;
    uint current_offset;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_hw4_parameters_t;

extern context_t context;

void esc_hw4_task(void *parameters);

#endif