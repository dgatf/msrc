#ifndef FUEL_METER_H
#define FUEL_METER_H

#include "common.h"

typedef struct fuel_meter_parameters_t {
    float ml_per_pulse;
    //float alpha;
    float *consumption_instant; // ml/min
    float *consumption_total; // ml
} fuel_meter_parameters_t;

extern context_t context;

void fuel_meter_task(void *parameters);

#endif