#ifndef XGZP68XXD_H
#define XGZP68XXD_H

#include "common.h"

typedef struct xgzp68xxd_parameters_t {
    uint16_t k;
    float *temperature, *pressure;
} xgzp68xxd_parameters_t;

extern context_t context;

void xgzp68xxd_task(void *parameters);

#endif