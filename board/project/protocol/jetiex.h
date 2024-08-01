#ifndef JETIEX_H
#define JETIEX_H

#include "common.h"

typedef struct sensor_jetiex_t {
    uint8_t data_id;
    uint8_t type;
    uint8_t format;
    char text[32];
    char unit[8];
    float *value;

} sensor_jetiex_t;

extern context_t context;

void jetiex_task(void *parameters);

#endif