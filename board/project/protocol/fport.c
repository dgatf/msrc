#include "fport.h"

#include <math.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "airspeed.h"
#include "bmp180.h"
#include "bmp280.h"
#include "config.h"
#include "current.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "esc_hw5.h"
#include "esc_kontronik.h"
#include "esc_omp_m4.h"
#include "esc_openyge.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gpio.h"
#include "gps.h"
#include "ina3221.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "smartport.h"
#include "stdlib.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define FPORT_DELIM 0x7E
#define FPORT_ESC 0x7D
#define FPORT_XOR 0x20
#define FPORT_FRAME_MAX 96

static void process(smartport_parameters_t *parameter);

void fport_task(void *parameters) {
    smartport_parameters_t parameter;
    smartport_set_protocol(FPORT);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    smartport_set_semaphore(xSemaphoreCreateBinary());
    xSemaphoreTake(smartport_get_semaphore(), 0);
    smartport_set_config(&parameter);
    debug("\nFport init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(smartport_parameters_t *parameter) {
    uint length = uart0_available();
    if (length < 3 || length > 128) return;

    uint8_t data[128];
    uart0_read_bytes(data, MIN(length, 128));
    
    // Byte stuffing in-place
    uint8_t delta = 0;
    uint i;
    for (i = 0; i < length; i++) {
        data[i] = data[i + delta];
        if (data[i] == 0x7D) {
            delta++;
            data[i] = data[i + delta] ^ 0x20;
        }
    }
    debug("\nFPORT (%u) < ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(data, length - delta, "0x%X ");

    uint idx = 0;
    if (data[1] != 0x08) {
        // Skip to next frame based on LEN
        idx = data[1] + 5;
    }
    if (idx + 11 != length - delta)
        return;  // Expecting exactly one frame with LEN=8 (plus LEN and CRC, ignoring RSSI if present)
    if (data[idx] == 0x08 && data[idx + 2] == 0x10) {
        xSemaphoreGive(smartport_get_semaphore());
        // FBUS requires a small turnaround delay before replying (RTOS tick = 2ms).
        vTaskDelay(pdMS_TO_TICKS(2));
        xSemaphoreTake(smartport_get_semaphore(), 0);
    }
}

