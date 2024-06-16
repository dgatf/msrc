#ifndef CURRENT_H
#define CURRENT_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "common.h"
#include "config.h"
#include "auto_offset.h"

typedef struct current_parameters_t
{
    uint8_t adc_num;
    float alpha, multiplier, offset;
    bool auto_offset;
    float *current, *consumption, *voltage;
} current_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;
extern uint8_t debug;

void current_task(void *parameters);

#endif