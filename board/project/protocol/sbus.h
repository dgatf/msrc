#ifndef SBUS_H
#define SBUS_H

#include "common.h"

typedef struct sensor_sbus_t {
    uint8_t data_id;
    float *value;
} sensor_sbus_t;

extern context_t context;

void sbus_task(void *parameters);

#endif
