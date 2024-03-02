#ifndef DISTANCE_H
#define DISTANCE_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "config.h"

#define DISTANCE_INIT_DELAY_MS 1000
#define DISTANCE_INTERVAL_MS 500

typedef struct distance_parameters_t
{
    float *distance, *latitude, *longitude, *altitude, *sat;

} distance_parameters_t;

extern uint8_t debug;

void distance_task(void *parameters);

#endif