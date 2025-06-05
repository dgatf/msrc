#ifndef UART_RX
#define UART_RX

#include "uart_rx.pio.h"

typedef void (*uart_rx_handler_t)(uint8_t data);

uint uart_rx_init(PIO pio, uint pin, uint baudrate, uint irq);
void uart_rx_set_handler(uart_rx_handler_t handler);
void uart_rx_remove(void);

#endif
