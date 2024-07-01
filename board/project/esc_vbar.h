#ifndef ESC_VBAR_H
#define ESC_VBAR_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "uart.h"
#include "cell_count.h"
#include "auto_offset.h"
#include "common.h"
#include "config.h"

#include "uart_pio.h"

#define ESC_VBAR_TIMEOUT_US 2000
#define ESC_VBAR_PACKET_LENGHT 32

typedef struct esc_vbar_parameters_t
{
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *temperature_motor, *voltage_bec, *current_bec, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_vbar_parameters_t;

extern TaskHandle_t pwm_out_task_handle, receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;
extern uint8_t debug;

void esc_vbar_task(void *parameters);

#endif