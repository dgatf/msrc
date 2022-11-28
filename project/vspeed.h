#ifndef VSPEED_H
#define VSPEED_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "config.h"

#define VSPEED_INIT_DELAY_MS 5000
#define VSPEED_INTERVAL 500

typedef struct vspeed_parameters_t
{
    uint interval;
    float *altitude, *vspeed;
} vspeed_parameters_t;

extern uint8_t debug;

void vspeed_task(void *parameters);

#endif