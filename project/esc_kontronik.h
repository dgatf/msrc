#ifndef ESC_KONTRONIK_H
#define ESC_KONTRONIK_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "uart.h"
#include "cell_count.h"
#include "common.h"
#include "config.h"

#define ESC_KONTRONIK_TIMEOUT 10000
#define ESC_KONTRONIK_PACKET_LENGHT 35

typedef struct esc_kontronik_parameters_t
{
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *voltage_bec, *current_bec, *temperature_fet, *temperature_bec, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_kontronik_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;
extern uint8_t debug;

void esc_kontronik_task(void *parameters);

#endif