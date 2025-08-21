#ifndef ADS7830_H
#define ADS7830_H

#include "common.h"

typedef struct ads7830_parameters_t {
    float alpha_voltage;
    uint8_t address;
    uint8_t *cell_count;
    float *cell[4];
} ads7830_parameters_t;

extern context_t context;

void ads7830_task(void *parameters);

#endif