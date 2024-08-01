#ifndef MULTIPLEX_H
#define MULTIPLEX_H

#include "common.h"

typedef struct sensor_multiplex_t {
    uint8_t data_id;
    float *value;
} sensor_multiplex_t;

extern context_t context;

void multiplex_task(void *parameters);

#endif