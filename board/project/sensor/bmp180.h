#ifndef BMP180_H
#define BMP180_H

#include "common.h"

typedef struct bmp180_parameters_t {
    float alpha_vario;
    bool auto_offset;
    float *temperature, *pressure, *altitude, *vspeed;
} bmp180_parameters_t;

typedef struct bmp180_calibration_t {
    int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;
} bmp180_calibration_t;

extern context_t context;

void bmp180_task(void *parameters);

#endif