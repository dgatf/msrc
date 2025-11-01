#include "jetiex_sensor.h"

#include <math.h>
#include <stdio.h>

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
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gps.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "stdlib.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "xgzp68xxd.h"

#define JETIEX_SENSOR_INTERVAL_MS 100

static int64_t alarm_packet(alarm_id_t id, void *user_data);
static void process(uint *baudrate, sensor_jetiex_t **sensor);
static void send_packet(sensor_jetiex_t **sensor);
static int64_t timeout_callback(alarm_id_t id, void *parameters);

void jetiex_sensor_task(void *parameters) {
    uint baudrate = 9600;
    sensor_jetiex_t *sensor[32] = {NULL};
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart_pio_begin(baudrate, UART_RECEIVER_TX, UART_RECEIVER_RX, 0, pio1, PIO1_IRQ_0, 9, 2,
                   UART_PARITY_ODD);
    jeti_set_config(sensor);
    add_alarm_in_us(0, alarm_packet, NULL, true);
    debug("\nJeti Ex Sensor init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        send_packet(sensor);
    }
}

static void send_packet(sensor_jetiex_t **sensor) {
    static uint8_t packet_count = 0;
    uint8_t buffer[36] = {0};
    uint length_telemetry_buffer = jeti_create_telemetry_buffer(buffer, packet_count % 16, sensor);
    uart_pio_write(0x7E);
    for (uint8_t i = 0; i < length_telemetry_buffer; i++) {
        uart_pio_write((uint32_t)buffer[i] | 0x100);
    }
    debug("\nJeti Ex Sensor %s (%u) > ", packet_count % 16 ? "Values " : "Text ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(buffer, length_telemetry_buffer, "0x%X ");
    packet_count++;

    // blink led
    vTaskResume(context.led_task_handle);
}

static int64_t alarm_packet(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveIndexedFromISR(context.receiver_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return JETIEX_SENSOR_INTERVAL_MS * 1000;
}
