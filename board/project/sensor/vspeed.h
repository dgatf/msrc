#ifndef VSPEED_H
#define VSPEED_H

#include "common.h"

typedef struct vspeed_parameters_t {
    uint interval;
    float *altitude, *vspeed;
} vspeed_parameters_t;

extern context_t context;

void vspeed_task(void *parameters);

#endif