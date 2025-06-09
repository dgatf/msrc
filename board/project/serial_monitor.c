#include "serial_monitor.h"

#include <math.h>
#include <stdio.h>

#include "config.h"
#include "pico/stdlib.h"
#include "uart.h"
#include "uart_pio.h"

static void process(config_t *config);

void serial_monitor_task(void *parameters) {
    xTaskNotifyGive(context.receiver_task_handle);
    config_t *config = config_read();
    debug("\nSerial monitor init. GPIO: %uBaudrate: %u Stop bits: %u Parity: %u Inverted: %u Timeout (ms): %u",
          config->serial_monitor_gpio, config->serial_monitor_baudrate, config->serial_monitor_stop_bits,
          config->serial_monitor_parity, config->serial_monitor_inverted, config->serial_monitor_timeout_ms);
    switch (config->serial_monitor_gpio) {
        case 1:
            uart0_begin(config->serial_monitor_baudrate, 0, 1, config->serial_monitor_timeout_ms * 1000, 8,
                        config->serial_monitor_stop_bits, config->serial_monitor_parity,
                        config->serial_monitor_inverted, false);
            break;
        case 5:
            uart1_begin(config->serial_monitor_baudrate, 4, 5, config->serial_monitor_timeout_ms * 1000, 8,
                        config->serial_monitor_stop_bits, config->serial_monitor_parity,
                        config->serial_monitor_inverted, false);
            break;
        case 6:
            uart_pio_begin(config->serial_monitor_baudrate, UART_GPIO_NONE, 6, config->serial_monitor_timeout_ms * 1000,
                           pio0, PIO0_IRQ_0);
    }

    while (1) {
        // debug("\n>> %u", config->serial_monitor_gpio);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(config);
    }
}

static void process(config_t *config) {
    static uint ts = 0;
    uint length;
    uint8_t data[length];
    switch (config->serial_monitor_gpio) {
        case 1:
            length = uart0_available();
            if (!length) return;
            uart0_read_bytes(data, length);
            break;
        case 5:
            length = uart1_available();
            if (!length) return;
            uart1_read_bytes(data, length);
            break;
        case 6:
            length = uart_pio_available();
            if (!length) return;
            uart_pio_read_bytes(data, length);
            break;
    }
    if (config->serial_monitor_timeout_ms)
        debug("\nSerial monitor (%u). GPIO: %u Length: %u (%u ms): ", uxTaskGetStackHighWaterMark(NULL),
              config->serial_monitor_gpio, length, (time_us_32() - ts) / 1000);
    if (config->serial_monitor_format == FORMAT_HEX) {
        debug_buffer(data, length, " 0x%X");
    } else if (context.debug)
        debug_buffer(data, length, "%c");
}