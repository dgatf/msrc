#ifndef SIM_RX_H
#define SIM_RX_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "uart.h"
#include "config.h"
#include "hitec.h"
#include "xbus.h"

#define SIM_RX_INTERVAL_MS 1000 // ms
#define SIM_RX_TIMEOUT_MS 1 // ms

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t uart0_queue_handle, uart1_queue_handle;

typedef struct sim_rx_parameters_t
{
    rx_protocol_t rx_protocol;
} sim_rx_parameters_t;

void sim_rx_task(void *parameters);

#endif