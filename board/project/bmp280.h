#ifndef BMP280_H
#define BMP280_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "config.h"
#include "common.h"
#include "auto_offset.h"
#include "vspeed.h"

#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA

#define BMP280_OVERSAMPLING_X0 0
#define BMP280_OVERSAMPLING_X1 1
#define BMP280_OVERSAMPLING_X2 2
#define BMP280_OVERSAMPLING_X4 3
#define BMP280_OVERSAMPLING_X8 4
#define BMP280_OVERSAMPLING_X16 5

#define BMP280_SLEEP 0
#define BMP280_FORCED 1
#define BMP280_NORMAL 3

#define FILTER_OFF 0
#define FILTER_X2 1
#define FILTER_X4 2
#define FILTER_X8 3
#define FILTER_X16 4

#define STANDBY_MS_1 0x00
#define STANDBY_MS_63 0x01
#define STANDBY_MS_125 0x02
#define STANDBY_MS_250 0x03
#define STANDBY_MS_500 0x04
#define STANDBY_MS_1000 0x05
#define STANDBY_MS_2000 0x06
#define STANDBY_MS_4000 0x07

#define BMP280_VARIO_INTERVAL 500
#define BMP280_SENSOR_INTERVAL_MS 100 // min 30

typedef struct bmp280_parameters_t
{
    float alpha_vario;
    bool auto_offset;
    uint8_t address;
    uint8_t filter;
    float *temperature, *pressure, *altitude, *vspeed;
} bmp280_parameters_t;

typedef struct bmp280_calibration_t
{
    uint16_t T1, P1;
    int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calibration_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;

void bmp280_task(void *parameters);

#endif