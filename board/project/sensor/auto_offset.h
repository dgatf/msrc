#ifndef AUTO_OFFSET_H
#define AUTO_OFFSET_H

#include "common.h"

typedef struct auto_offset_float_parameters_t {
    uint delay;
    float *value;
    float *offset;

} auto_offset_float_parameters_t;

typedef struct auto_offset_int_parameters_t {
    uint delay;
    int *value;
    int *offset;

} auto_offset_int_parameters_t;

extern context_t context;

void auto_offset_float_task(void *parameters);
void auto_offset_int_task(void *parameters);

#endif