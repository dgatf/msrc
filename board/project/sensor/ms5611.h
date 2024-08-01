#ifndef MS5611_H
#define MS5611_H

#include "common.h"

typedef struct ms5611_parameters_t {
    float alpha_vario;
    bool auto_offset;
    uint8_t address;
    float *temperature, *pressure, *altitude, *vspeed;
} ms5611_parameters_t;

typedef struct ms5611_calibration_t {
    uint16_t C1, C2, C3, C4, C5, C6;
} ms5611_calibration_t;

extern context_t context;

void ms5611_task(void *parameters);

#endif