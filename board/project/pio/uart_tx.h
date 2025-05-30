#ifndef UART_TX
#define UART_TX

#include "uart_tx.pio.h"

uint uart_tx_init(PIO pio, uint pin, uint baudrate);
void uart_tx_write(uint8_t c);
void uart_tx_write_bytes(uint8_t *data, uint8_t length);
void uart_tx_remove(void);

#endif