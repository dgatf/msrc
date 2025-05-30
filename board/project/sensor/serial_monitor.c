#include "serial_monitor.h"

#include <math.h>
#include <stdio.h>

#include "config.h"
#include "pico/stdlib.h"
#include "uart.h"

static void process(uint gpio);

void serial_monitor_task(void *parameters) {
    xTaskNotifyGive(context.receiver_task_handle);
    config_t *config = config_read();
    debug("\nSerial monitor init. GPIO: %uBaudrate: %u Stop bits: %u Parity: %u Inverted: %u Timeout (ms): %u",
          config->serial_monitor_gpio, config->serial_monitor_baudrate, config->serial_monitor_stop_bits,
          config->serial_monitor_parity, config->serial_monitor_inverted, config->serial_monitor_timeout_ms);
    if (config->serial_monitor_gpio == 1)
        uart0_begin(
            config->serial_monitor_baudrate, config->serial_monitor_gpio - 1, config->serial_monitor_gpio, config->serial_monitor_timeout_ms * 1000,
            8, config->serial_monitor_stop_bits, config->serial_monitor_parity, config->serial_monitor_inverted, false);
    else
        uart1_begin(
            config->serial_monitor_baudrate, config->serial_monitor_gpio - 1, config->serial_monitor_gpio, config->serial_monitor_timeout_ms * 1000,
            8, config->serial_monitor_stop_bits, config->serial_monitor_parity, config->serial_monitor_inverted, false);
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(config->serial_monitor_gpio);
    }
}

static void process(uint gpio) {
    static uint ts = 0;
    uint length = gpio == 1 ? uart0_available() : uart1_available();
    if (length) {
        uint8_t data[length];
        if (gpio == 1) uart0_read_bytes(data, length);
        else uart1_read_bytes(data, length);
        debug("\nSerial monitor (%u). Length: %u (%u ms): ", uxTaskGetStackHighWaterMark(NULL), length,
              (time_us_32() - ts) / 1000);
        debug_buffer(data, length, " 0x%X");
    }
}
