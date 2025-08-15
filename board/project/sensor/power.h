#ifndef POWER_H
#define POWER_H

#include "common.h"

typedef struct power_parameters_t {
    uint8_t adc_current_num;
    uint8_t adc_voltage_num;
    uint8_t rate;
    float alpha, multiplier_current, multiplier_voltage, offset;
    bool auto_offset;
    float *current, *consumption, *voltage_current, *voltage, *power, *power_consumption;
} power_parameters_t;

extern context_t context;

void power_task(void *parameters);

#endif