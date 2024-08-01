#ifndef UART_PIO_H
#define UART_PIO_H

#include "common.h"
#include "uart_rx.h"

#define UART_PIO_BUFFER_SIZE 2048

extern context_t context;

void uart_pio_begin(uint baudrate, uint gpio_rx, uint timeout, PIO pio, uint irq);
uint8_t uart_pio_read();
void uart_pio_read_bytes(uint8_t *data, uint8_t lenght);
uint8_t uart_pio_available();
uint uart_pio_get_time_elapsed();

#endif