#ifndef VOLTAGE_H
#define VOLTAGE_H

#include "common.h"

typedef struct voltage_parameters_t {
    uint8_t adc_num;
    uint8_t rate;
    float alpha, multiplier;
    float *voltage;
} voltage_parameters_t;

extern context_t context;

void voltage_task(void *parameters);

#endif