#ifndef BMP180_H
#define BMP180_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "config.h"
#include "common.h"
#include "auto_offset.h"
#include "vspeed.h"

#define BMP180_REGISTER_DIG_AC1 0xAA
#define BMP180_REGISTER_DIG_AC2 0xAC
#define BMP180_REGISTER_DIG_AC3 0xAD
#define BMP180_REGISTER_DIG_AC4 0xB0
#define BMP180_REGISTER_DIG_AC5 0xB2
#define BMP180_REGISTER_DIG_AC6 0xB4
#define BMP180_REGISTER_DIG_B1 0xB6
#define BMP180_REGISTER_DIG_B2 0xB8
#define BMP180_REGISTER_DIG_MB 0xBA
#define BMP180_REGISTER_DIG_MC 0xBC
#define BMP180_REGISTER_DIG_MD 0xBE

#define BMP180_REGISTER_CONTROL 0xF4
#define BMP180_REGISTER_CHIPID 0xD0
#define BMP180_REGISTER_SOFTRESET 0xE0
#define BMP180_REGISTER_DATA 0xF6

#define BMP180_READ_TEMPERATURE 0x2E
#define BMP180_READ_PRESSURE 0x34

#define BMP180_OVERSAMPLING_0 0
#define BMP180_OVERSAMPLING_1 1
#define BMP180_OVERSAMPLING_2 2
#define BMP180_OVERSAMPLING_3 3

#define BMP180_READ_PRESSURE_OVERSAMPLING_1 0x74
#define BMP180_READ_PRESSURE_OVERSAMPLING_2 0xB4
#define BMP180_READ_PRESSURE_OVERSAMPLING_3 0xF4

#define BMP180_VARIO_INTERVAL 500
#define BMP180_PRESSURE_INTERVAL_MS 100 // min 30
#define BMP180_TEMPERATURE_INTERVAL_MS 100 // min 10


typedef struct bmp180_parameters_t
{
    float alpha_vario;
    bool auto_offset;
    uint8_t address;
    float *temperature, *pressure, *altitude, *vspeed;
} bmp180_parameters_t;

typedef struct bmp180_calibration_t
{
    int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;
} bmp180_calibration_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;

void bmp180_task(void *parameters);

#endif