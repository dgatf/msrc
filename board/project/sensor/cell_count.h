#ifndef CELL_COUNT_H
#define CELL_COUNT_H

#include "common.h"

typedef struct cell_count_parameters_t {
    uint delay;
    float *voltage;
    uint8_t *cell_count;
} cell_count_parameters_t;

extern context_t context;

void cell_count_task(void *parameters);

#endif