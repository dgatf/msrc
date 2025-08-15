#ifndef COMMON_H
#define COMMON_H

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "constants.h"
#include "pico/stdlib.h"
#include "pico/types.h"
#include "shared.h"

/*
   Debug
   Disconnect Vcc from the RC model to the board before connecting USB
   Telemetry may not work properly in debug mode
*/

#define MSRC_DEBUG 0  // 0 = no debug, 1 = debug level 1, 2 = debug level 2

// #define SIM_RX
// #define SIM_SENSORS

// #define SIM_SMARTPORT_SEND_CONFIG_LUA
// #define SIM_SMARTPORT_RECEIVE_CONFIG_LUA
// #define SIM_SMARTPORT_SEND_SENSOR_ID
// #define SIM_SMARTPORT_RECEIVE_SENSOR_ID

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)
#define swap_24(value) (((value & 0xFF) << 16) | (value & 0xFF00) | (value & 0xFF0000) >> 16)
#define swap_32(value) \
    (((value & 0xFF) << 24) | ((value & 0xFF00) << 8) | ((value & 0xFF0000) >> 8) | ((value & 0xFF000000) >> 24))
    
#define debug(...) \
    if (context.debug == 1) printf(__VA_ARGS__)
#define debug_buffer(buffer, length, format) \
    if (context.debug)                       \
        for (int i = 0; i < (length); i++) printf((format), (buffer)[i])
#define debug2(...) \
    if (context.debug == 2) printf(__VA_ARGS__)
#define debug_buffer2(buffer, length, format) \
    if (context.debug == 2)                   \
        for (int i = 0; i < (length); i++) printf((format), (buffer)[i])

typedef struct context_t {
    TaskHandle_t pwm_out_task_handle, uart0_notify_task_handle, uart1_notify_task_handle, uart_pio_notify_task_handle,
        receiver_task_handle, led_task_handle, usb_task_handle;
    QueueHandle_t uart0_queue_handle, uart1_queue_handle, uart_rx_pio_queue_handle, uart_tx_pio_queue_handle, tasks_queue_handle,
        sensors_queue_handle;
    alarm_pool_t *uart_alarm_pool;
    uint8_t debug, led_cycles;
    uint16_t led_cycle_duration;
} context_t;

float get_average(float alpha, float prev_value, float new_value);
float get_consumption(float current, uint16_t current_max, uint32_t *timestamp);
float get_energy(float power, uint16_t power_max, uint32_t *timestamp_power);
float voltage_read(uint8_t adc_num);
float get_altitude(float pressure, float temperature, float P0);
void get_vspeed(float *vspeed, float altitude, uint interval);
void get_vspeed_gps(float *vspeed, float altitude, uint interval);

#endif