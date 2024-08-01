#ifndef IBUS_H
#define IBUS_H

#include "common.h"

typedef struct sensor_ibus_t {
    uint8_t data_id;
    uint8_t type;
    float *value;
} sensor_ibus_t;

extern context_t context;

void ibus_task(void *parameters);

#endif