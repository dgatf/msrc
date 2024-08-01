#ifndef ESC_HW3_H
#define ESC_HW3_H

#include "common.h"

typedef struct esc_hw3_parameters_t {
    float multiplier;
    float alpha;
    float *rpm;
} esc_hw3_parameters_t;

extern context_t context;

void esc_hw3_task(void *parameters);

#endif