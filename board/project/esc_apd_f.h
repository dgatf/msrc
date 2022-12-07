#ifndef ESC_APD_F_H
#define ESC_APD_F_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "uart.h"
#include "cell_count.h"
#include "common.h"
#include "config.h"

#define ESC_APD_F_TIMEOUT_US 1000
#define ESC_APD_F_PACKET_LENGHT 12
#define ESC_KISS_PACKET_LENGHT 10

typedef struct esc_apd_f_parameters_t
{
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_apd_f_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;
extern uint8_t debug;

void esc_apd_f_task(void *parameters);

#endif