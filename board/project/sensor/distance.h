#ifndef DISTANCE_H
#define DISTANCE_H

#include "common.h"

typedef struct distance_parameters_t {
    float *distance, *latitude, *longitude, *altitude, *sat;

} distance_parameters_t;

extern context_t context;

void distance_task(void *parameters);

#endif