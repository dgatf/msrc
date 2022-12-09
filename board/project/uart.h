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

void uart0_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uint parity, bool inverted);
uint8_t uart0_read();
void uart0_read_bytes(uint8_t *data, uint8_t lenght);
uint8_t uart0_available();
uint uart0_get_time_elapsed();
void uart0_write(uint8_t data);
void uart0_write_bytes(uint8_t *data, uint8_t lenght);

void uart1_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uint parity, bool inverted);
uint8_t uart1_read();
void uart1_read_bytes(uint8_t *data, uint8_t lenght);
uint8_t uart1_available();
uint uart1_get_time_elapsed();
void uart1_write(uint8_t data);
void uart1_write_bytes(uint8_t *data, uint8_t lenght);

void uart0_set_timestamp();
void uart1_set_timestamp();

#endif