#ifndef INA3221_H
#define INA3221_H

#include "common.h"

typedef struct ina3221_parameters_t {
    uint8_t i2c_address;
    uint8_t filter;
    uint8_t conversion_time;
    uint8_t cell_count;
    float *cell[3];
    float *cell_prev;
} ina3221_parameters_t;

extern context_t context;

void ina3221_task(void *parameters);

#endif