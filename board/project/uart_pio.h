#ifndef UART_PIO_H
#define UART_PIO_H

#include "common.h"
#include "uart_rx.h"
#include "uart_tx.h"

#define UART_PIO_BUFFER_SIZE 2048
#define UART_GPIO_NONE -1

extern context_t context;

void uart_pio_begin(uint baudrate, int gpio_tx, int gpio_rx, uint timeout, PIO pio, uint irq, uint8_t data_bits,
                    uint8_t stop_bits, uint8_t parity);
uint8_t uart_pio_read(void);
void uart_pio_read_bytes(uint8_t *data, uint8_t lenght);
void uart_pio_write(uint32_t c);
void uart_pio_write_bytes(void *data, uint8_t lenght);
uint8_t uart_pio_available(void);
uint8_t uart_pio_tx_available(void);
void uart_pio_remove(void);

#endif