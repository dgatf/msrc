#ifndef ESC_APD_HV_H
#define ESC_APD_HV_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "uart.h"
#include "cell_count.h"
#include "common.h"
#include "config.h"

#define ESC_APD_HV_TIMEOUT 10000
#define ESC_APD_HV_PACKET_LENGHT 22

typedef struct esc_apd_hv_parameters_t
{
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_apd_hv_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;
extern uint8_t debug;

void esc_apd_hv_task(void *parameters);

#endif