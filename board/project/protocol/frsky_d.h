#ifndef FRSKY_D_H
#define FRSKY_D_H

#include "common.h"

typedef struct frsky_d_sensor_parameters_t {
    uint8_t data_id;
    float *value;
    uint16_t rate;

} frsky_d_sensor_parameters_t;

typedef struct frsky_d_sensor_cell_parameters_t {
    float *voltage;
    uint8_t *count;
    uint16_t rate;

} frsky_d_sensor_cell_parameters_t;

extern context_t context;

void frsky_d_task(void *parameters);

#endif