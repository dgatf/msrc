#ifndef UART_TX
#define UART_TX

#include "uart_tx.pio.h"

uint uart_tx_init(PIO pio, uint pin, uint baudrate, uint data_bits, uint stop_bits, uint parity);
void uart_tx_write(uint32_t c);
void uart_tx_write_bytes(void *data, uint8_t length);
void uart_tx_remove(void);

#endif