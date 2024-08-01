#ifndef AIRSPEED_H
#define AIRSPEED_H

#include "common.h"

typedef struct airspeed_parameters_t {
    uint8_t adc_num;
    uint8_t rate;
    float alpha;
    float *airspeed;
} airspeed_parameters_t;

extern context_t context;

void airspeed_task(void *parameters);

#endif