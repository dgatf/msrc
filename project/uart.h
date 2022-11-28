#ifndef UART_H
#define UART_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART0_BUFFER_SIZE 256
#define UART1_BUFFER_SIZE 256

extern TaskHandle_t uart0_notify_task_handle, uart1_notify_task_handle;
extern QueueHandle_t uart0_queue_handle, uart1_queue_handle;
extern alarm_pool_t *uart_alarm_pool;

void uart_begin(uart_inst_t *uart, uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uint parity, bool inverted);
uint8_t uart_read(uart_inst_t *uart);
void uart_read_bytes(uart_inst_t *uart, uint8_t *data, uint8_t lenght);
uint8_t uart_available(uart_inst_t *uart);
uint uart_get_time_elapsed(uart_inst_t *uart);
void uart_write(uart_inst_t *uart, uint8_t data);
void uart_write_bytes(uart_inst_t *uart, uint8_t *data, uint8_t lenght);

void uart_set_timestamp(uart_inst_t *uart);

#endif