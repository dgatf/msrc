#ifndef CELL_COUNT_H
#define CELL_COUNT_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "config.h"

typedef struct cell_count_parameters_t
{
    uint delay;
    float *voltage;
    uint8_t *cell_count;
} cell_count_parameters_t;

extern uint8_t debug;

void cell_count_task(void *parameters);

#endif