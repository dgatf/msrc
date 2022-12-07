#ifndef ESC_HW3_H
#define ESC_HW3_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "config.h"
#include "uart.h"
#include "common.h"

#define ESC_HW3_TIMEOUT_US 1000
#define ESC_HW3_PACKET_LENGHT 10
#define ESC_HW3_NO_SIGNAL_TIMEOUT_MS 500

typedef struct esc_hw3_parameters_t
{
    float multiplier;
    float alpha;
    float *rpm;
} esc_hw3_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern uint8_t debug;

void esc_hw3_task(void *parameters);

#endif