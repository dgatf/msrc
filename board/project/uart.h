#ifndef UART_H
#define UART_H

#include "common.h"

extern context_t context;

void uart0_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uart_parity_t parity,
                 bool inverted, bool half_duplex);
uint8_t uart0_read();
void uart0_read_bytes(uint8_t *data, uint8_t lenght);
uint8_t uart0_available();
uint uart0_get_time_elapsed();
void uart0_write(uint8_t data);
void uart0_write_bytes(uint8_t *data, uint8_t lenght);

void uart1_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uart_parity_t parity,
                 bool inverted, bool half_duplex);
uint8_t uart1_read();
void uart1_read_bytes(uint8_t *data, uint8_t lenght);
uint8_t uart1_available();
uint uart1_get_time_elapsed();
void uart1_write(uint8_t data);
void uart1_write_bytes(uint8_t *data, uint8_t lenght);

void uart0_set_timestamp();
void uart1_set_timestamp();

#endif