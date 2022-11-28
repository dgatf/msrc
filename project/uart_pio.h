#ifndef UART_PIO_H
#define UART_PIO_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "uart_rx.h"

#define UART_PIO_BUFFER_SIZE 256

extern TaskHandle_t uart_pio_task_handle;
extern QueueHandle_t uart_pio_queue_handle;
extern alarm_pool_t *uart_alarm_pool;

void uart_pio_begin(uint baudrate, uint gpio_rx, uint timeout, PIO pio, uint irq);
uint8_t uart_pio_read();
void uart_pio_read_bytes(uint8_t *data, uint8_t lenght);
uint8_t uart_pio_available();
uint uart_pio_get_time_elapsed();

#endif