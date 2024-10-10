#ifndef ESC_HW4_H
#define ESC_HW4_H

#include "common.h"

/* ESCHW4_DIVISOR and ESCHW4_AMPGAIN values

Divisor: Cells range

3-6S (LV): divisor = 11
3-8S (LV v2): divisor = 15.4
5-12s (HV): divisor = 21

Gain: Amperage

60A: gain = 6
80A: gain = 7.8
100A: gain = 9(1)
120A: gain = 10
130A: gain = 11.3(1)
150A: gain = 12.9(1)
160A: gain = 13.7(1)
200A: gain = 16.9

(1) Extrapolated from confirmed models

Current multiplier = 1 / (ampgain* 0.25/1000) = 4000 / ampgain

*/

typedef struct esc_hw4_parameters_t {
    float rpm_multiplier;
    bool pwm_out;
    bool init_delay;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float divisor, current_multiplier, current_thresold, current_max;
    bool current_is_manual_offset;
    float current_offset;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_hw4_parameters_t;

extern context_t context;

void esc_hw4_task(void *parameters);

#endif