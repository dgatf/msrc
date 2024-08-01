#ifndef CURRENT_H
#define CURRENT_H

#include "common.h"

typedef struct current_parameters_t {
    uint8_t adc_num;
    uint8_t rate;
    float alpha, multiplier, offset;
    bool auto_offset;
    float *current, *consumption, *voltage;
} current_parameters_t;

extern context_t context;

void current_task(void *parameters);

#endif