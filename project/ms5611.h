#ifndef MS5611_H
#define MS5611_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>

#include "pico/stdlib.h"

#include "config.h"
#include "common.h"
#include "auto_offset.h"
#include "vspeed.h"

#define MS5611_CMD_ADC_READ 0x00
#define MS5611_CMD_RESET 0x1E
#define MS5611_CMD_CONV_D1 0x40
#define MS5611_CMD_CONV_D2 0x50

// 0 0xA0,0xA1 - reserved
// 1 0xA2,0xA3 - C1
// 2 0xA4,0xA5 - C2
// 3 0xA6,0xA7 - C3
// 4 0xA8,0xA9 - C4
// 5 0xAA,0xAB - C5
// 6 0xAC,0xAD - C6
// 7 0xAE - CRC (4bits)

#define MS5611_CMD_READ_PROM 0xA0

#define MS5611_OVERSAMPLING_4096 0x08
#define MS5611_OVERSAMPLING_2048 0x06
#define MS5611_OVERSAMPLING_1024 0x04
#define MS5611_OVERSAMPLING_512 0x02
#define MS5611_OVERSAMPLING_256 0x00

#define MS5611_VARIO_INTERVAL 500
#define MS5611_SENSOR_INTERVAL_MS 10

typedef struct ms5611_parameters_t
{
    float alpha_vario;
    bool auto_offset;
    uint8_t address;
    float *temperature, *pressure, *altitude, *vspeed;
} ms5611_parameters_t;

typedef struct ms5611_calibration_t
{
    uint16_t C1, C2, C3, C4, C5, C6;
} ms5611_calibration_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;
extern uint8_t debug;

void ms5611_task(void *parameters);

#endif