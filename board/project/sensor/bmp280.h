#ifndef BMP280_H
#define BMP280_H

#include "common.h"

typedef struct bmp280_parameters_t {
    float alpha_vario;
    bool auto_offset;
    uint8_t address;
    uint8_t filter;
    float *temperature, *pressure, *altitude, *vspeed;
} bmp280_parameters_t;

typedef struct bmp280_calibration_t {
    uint16_t T1, P1;
    int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calibration_t;

extern context_t context;

void bmp280_task(void *parameters);

#endif