#include "usb.h"

#include <queue.h>
#include <stdio.h>

#include "config.h"
#include "pico/stdlib.h"
#include "string.h"

#define AIRCR_Register (*((volatile uint32_t *)(PPB_BASE + 0x0ED0C)))

#define USB_BUFFER_LENGTH 256
#define USB_INTERVAL_MS 1000

static uint8_t buffer_rx[USB_BUFFER_LENGTH];

static int read_usb();
static void process_usb();

void usb_task() {
    while (1) {
        int length = read_usb();
        if (length) process_usb(length);
        debug("\nUSB (%u)", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(USB_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void process_usb(int lenght) {
    debug("\nUSB. Processing (%i) 0x%X 0x%X", lenght, buffer_rx[0], buffer_rx[1]);

    /*
    header - 0x30
    command - 0x30 - read config from usb
              0x31 - request to send config to usb
              0x32 - answer to send config
              0x33 - debug on
              0x34 - debug off
    */

    if (buffer_rx[0] == 0x30) {
        if (buffer_rx[1] == 0x30 &&
            lenght == sizeof(config_t) + 2) {  // read config from usb, write to flash and reboot

            config_t *config = (config_t *)(buffer_rx + 2);
            if (config->version == CONFIG_VERSION) {
                context.led_cycles = 3;
                context.led_cycle_duration = 1000;
                vTaskResume(context.led_task_handle);
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                context.led_cycles = 1;
                context.led_cycle_duration = 6;
                vTaskResume(context.led_task_handle);
                config_write(config);
                debug("\nUSB. Updated config");
                // sleep_ms(1000);
                // AIRCR_Register = 0x5FA0004;
            } else
                debug("\nUSB. Incompatible config version");
        } else if (buffer_rx[1] == 0x31 && lenght == 2) {  // send config
            uint8_t debug_state = context.debug;
            context.debug = 0;
            context.led_cycles = 2;
            context.led_cycle_duration = 1000;
            vTaskResume(context.led_task_handle);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            context.led_cycles = 1;
            context.led_cycle_duration = 6;
            vTaskResume(context.led_task_handle);
            config_t *config = config_read();
            putchar_raw(0x30);
            putchar_raw(0x32);
            uint8_t data[sizeof(config_t)];
            memcpy(data, config, sizeof(config_t));
            for (int i = 0; i < sizeof(config_t); i++) putchar_raw(data[i]);
            if (debug_state) vTaskDelay(1000 / portTICK_PERIOD_MS);
            context.debug = debug_state;
            debug("\nUSB. Send config");
        } else if (buffer_rx[1] == 0x33 && lenght == 2) {  // debug enable
            context.debug = 1;
            debug("\nUSB. Debug enabled");
        } else if (buffer_rx[1] == 0x34 && lenght == 2) {  // debug disable
            context.debug = 0;
            debug("\nUSB. Debug disabled");
        }
    }
}

static int read_usb() {
    int buffer_index = 0;
    while (1) {
        int c = getchar_timeout_us(1000);
        if (c != PICO_ERROR_TIMEOUT && buffer_index < USB_BUFFER_LENGTH) {
            buffer_rx[buffer_index++] = (c & 0xFF);
        } else {
            break;
        }
    }
    return buffer_index;
}
