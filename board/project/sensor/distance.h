#ifndef DISTANCE_H
#define DISTANCE_H

#include "common.h"

typedef struct distance_parameters_t {
    float *distance, *latitude, *longitude, *altitude, *sat, *fix, *hdop; // fix internal msrc: 0 no fix, 1 2D fix, 2 3D fix
    bool *home_set;

} distance_parameters_t;

extern context_t context;

void distance_task(void *parameters);

#endif