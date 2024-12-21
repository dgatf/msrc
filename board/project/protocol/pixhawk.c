#include "pixhawk.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>

void pixhawk_task(void *parameters) {
    uart0_begin(57600, UART_RECEIVER_TX, UART_RECEIVER_RX, 1000, 8, 1, UART_PARITY_NONE, false);
    while (1) {
        if (uart0_available()) {
            uint8_t data = uart0_read();
            // Process PixHawk telemetry data
            // Add your data parsing and processing logic here
        }
    }
}
