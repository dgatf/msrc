#ifndef VOLTAGE_H
#define VOLTAGE_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "common.h"
#include "config.h"

typedef struct voltage_parameters_t
{
    uint8_t adc_num;
    float alpha, multiplier;
    float *voltage;
} voltage_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern uint8_t debug;

void voltage_task(void *parameters);

#endif