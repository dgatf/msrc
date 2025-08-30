#ifndef AIRSPEED_H
#define AIRSPEED_H

#include "common.h"

typedef struct airspeed_parameters_t {
    uint8_t adc_num;
    uint8_t rate;
    float alpha;
    float offset;        // accuracy = ±6.25% VFSS
    float vcc;           // nominal is 5V
    float *temperature;  // air temp (input from baro)
    float *pressure;     // air pressure (input from baro)
    float *airspeed;
} airspeed_parameters_t;

extern context_t context;

void airspeed_task(void *parameters);

#endif