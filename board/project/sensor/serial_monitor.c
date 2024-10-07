#include <math.h>
#include <stdio.h>

#include "serial_monitor.h"
#include "pico/stdlib.h"
#include "uart.h"
#include "config.h"

static void process(void);

void serial_monitor_task(void *parameters) {
    xTaskNotifyGive(context.receiver_task_handle);
    config_t *config = config_read();
    debug("\nSerial monitor init. Baudrate: %u Stop bits: %u Parity: %u Inverted: %u Timeout (ms): %u", config->serial_monitor_baudrate, config->serial_monitor_stop_bits, config->serial_monitor_parity, config->serial_monitor_inverted, config->serial_monitor_timeout_ms);
    uart1_begin(config->serial_monitor_baudrate, UART1_TX_GPIO, UART_ESC_RX, config->serial_monitor_timeout_ms * 1000, 8, 1, config->serial_monitor_parity, config->serial_monitor_inverted);
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static void process(void) {
    static uint ts = 0;
    uint length = uart1_available();
    if (length) {
        uint8_t data[length];
        uart1_read_bytes(data, length);
        debug("\nSerial monitor (%u). Length: %u (%u ms): ", uxTaskGetStackHighWaterMark(NULL), length, (time_us_32() - ts) / 1000);
        debug_buffer(data, length, " 0x%X");
        ts = time_us_32();
    }
}
