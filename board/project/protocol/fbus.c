#include "fbus.h"

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

static void process(smartport_parameters_t *parameter);

void fbus_task(void *parameters) {
    smartport_parameters_t parameter;
    smartport_set_protocol(FBUS);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    smartport_set_semaphore(xSemaphoreCreateBinary());
    xSemaphoreTake(smartport_get_semaphore(), 0);
    smartport_set_config(&parameter);
    debug("\nFbus init");
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

    debug("\nFBUS (%u) < ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(data, length, "0x%X ");

    uint idx = 0;
    if (data[0] != 0x08) {
        // Skip to next frame based on LEN
        idx = data[0] + 2;
        // Control frame (0xFF ...), RSSI byte present but not counted in LEN
        if (data[1] == 0xFF) idx++;
    }
    if (idx + 10 != length)
        return;  // Expecting exactly one frame with LEN=8 (plus LEN and CRC, ignoring RSSI if present)
    if (data[idx] == 0x08 && data[idx + 1] == smartport_sensor_id_to_crc(smartport_get_sensor_id()) && data[idx + 2] == 0x10) {
        xSemaphoreGive(smartport_get_semaphore());
        // FBUS requires a small turnaround delay before replying (RTOS tick = 2ms).
        vTaskDelay(pdMS_TO_TICKS(2));
        xSemaphoreTake(smartport_get_semaphore(), 0);
    }
}

