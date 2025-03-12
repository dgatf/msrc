#include "serial_monitor.h"

#include <math.h>
#include <stdio.h>

#include "config.h"
#include "pico/stdlib.h"
#include "uart.h"

static void process(void);

void serial_monitor_task(void *parameters) {
    xTaskNotifyGive(context.receiver_task_handle);
    config_t *config = config_read();
    debug("\nSerial monitor init. Baudrate: %u Stop bits: %u Parity: %u Inverted: %u Timeout (ms): %u",
          config->serial_monitor_baudrate, config->serial_monitor_stop_bits, config->serial_monitor_parity,
          config->serial_monitor_inverted, config->serial_monitor_timeout_ms);
    uart1_begin(config->serial_monitor_baudrate, UART1_TX_GPIO, UART_ESC_RX, config->serial_monitor_timeout_ms * 1000,
                8, 1, config->serial_monitor_parity, config->serial_monitor_inverted, false);
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
        debug("\nSerial monitor (%u). Length: %u (%u ms): ", uxTaskGetStackHighWaterMark(NULL), length,
              (time_us_32() - ts) / 1000);
        debug_buffer(data, length, " 0x%X");
        /*uint16_t ch1, ch2, ch3, ch4, temp1_0, temp2_0, temp1_1, temp2_1, rpm_3, rpm_4, type, volt_5;
        ch1 = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
        ch2 = (uint16_t)(((uint16_t)data[3] << 8) | data[4]);
        ch3 = (uint16_t)(((uint16_t)data[5] << 8) | data[6]);
        ch4 = (uint16_t)(((uint16_t)data[7] << 8) | data[8]);
        type = data[11];
        if (type == 0) {
            temp1_0 = data[12];
            temp2_0 = data[13];
            debug("\nSanwa ESC. ch1: %u ch2: %u ch3: %u ch4: %u type: %u temp1_0: %u temp2_0: %u ", ch1, ch2, ch3, ch4,
                  type, temp1_0, temp2_0);
        }
        if (type == 1) {
            temp1_1 = data[12];
            temp2_1 = data[13];
            debug("\nSanwa ESC. ch1: %u ch2: %u ch3: %u ch4: %u type: %u temp1_1: %u temp2_1: %u", ch1, ch2, ch3, ch4,
                  type, temp1_1, temp2_1);
        }
        if (type == 3) {
            rpm_3 = (data[12] << 8) | data[13];
            debug("\nSanwa ESC. ch1: %u ch2: %u ch3: %u ch4: %u type: %u rpm_3: %u", ch1, ch2, ch3, ch4, type, rpm_3);
        }
        if (type == 4) {
            rpm_4 = (data[12] << 8) | data[13];
            debug("\nSanwa ESC. ch1: %u ch2: %u ch3: %u ch4: %u type: %u rpm_4: %u", ch1, ch2, ch3, ch4, type, rpm_4);
        }
        if (type == 5) {
            volt_5 = (((uint16_t)data[12] << 8) | data[13]);
            debug(
                "\nSanwa ESC. ch1: %u ch2: %u ch3: %u ch4: %u type: %u volt_5: %u",
                ch1, ch2, ch3, ch4, type, volt_5);
        }
        ts = time_us_32();*/
    }
}
