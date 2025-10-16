#ifndef MPU6050_H
#define MPU6050_H

#include "common.h"

typedef struct mpu6050_parameters_t {
    float alpha_gyro;
    uint8_t address, acc_scale, gyro_scale, gyro_weighting, filter;
    float *acc_x, *acc_y, *acc_z, *acc;
    float *roll, *pitch, *yaw;
} mpu6050_parameters_t;

extern context_t context;

void mpu6050_task(void *parameters);

#endif