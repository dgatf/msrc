#ifndef COMMON_H
#define COMMON_H

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "constants.h"
#include "pico/stdlib.h"
#include "pico/types.h"

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

typedef enum rx_protocol_t {
    RX_SMARTPORT,
    RX_FRSKY_D,
    RX_XBUS,
    RX_SRXL,
    RX_IBUS,
    RX_SBUS,
    RX_MULTIPLEX,
    RX_JETIEX,
    RX_HITEC,
    RX_SRXL2,
    SERIAL_MONITOR,
    RX_CRSF,
    RX_HOTT,
    RX_SANWA,
    RX_JR_PROPO
} rx_protocol_t;

typedef enum esc_protocol_t {
    ESC_NONE,
    ESC_HW3,
    ESC_HW4,
    ESC_PWM,
    ESC_CASTLE,
    ESC_KONTRONIK,
    ESC_APD_F,
    ESC_APD_HV,
    ESC_HW5
} esc_protocol_t;

typedef enum i2c_module_t { I2C_NONE, I2C_BMP280, I2C_MS5611, I2C_BMP180 } i2c_module_t;

typedef enum analog_current_type_t { CURRENT_TYPE_HALL, CURRENT_TYPE_SHUNT } analog_current_type_t;

typedef struct config_t {
    uint16_t version;
    enum rx_protocol_t rx_protocol;
    enum esc_protocol_t esc_protocol;
    bool enable_gps;
    uint32_t gps_baudrate;
    bool enable_analog_voltage;
    bool enable_analog_current;
    bool enable_analog_ntc;
    bool enable_analog_airspeed;
    enum i2c_module_t i2c_module;
    uint8_t i2c_address;
    float alpha_rpm;
    float alpha_voltage;
    float alpha_current;
    float alpha_temperature;
    float alpha_vario;
    float alpha_airspeed;
    uint16_t refresh_rate_rpm;
    uint16_t refresh_rate_voltage;
    uint16_t refresh_rate_current;
    uint16_t refresh_rate_temperature;
    uint16_t refresh_rate_gps;
    uint16_t refresh_rate_consumption;
    uint16_t refresh_rate_vario;
    uint16_t refresh_rate_airspeed;
    uint16_t refresh_rate_default;
    float analog_voltage_multiplier;
    enum analog_current_type_t analog_current_type;
    uint16_t spare0;
    float analog_current_quiescent_voltage;
    float analog_current_multiplier;
    float analog_current_offset;
    bool analog_current_autoffset;
    uint8_t pairOfPoles;
    uint8_t mainTeeth;
    uint8_t pinionTeeth;
    float rpm_multiplier;
    uint8_t bmp280_filter;
    bool enable_pwm_out;
    uint8_t smartport_sensor_id;
    uint16_t smartport_data_id;
    bool vario_auto_offset;
    bool xbus_clock_stretch;
    bool jeti_gps_speed_units_kmh;
    bool enable_esc_hw4_init_delay;
    uint16_t esc_hw4_init_delay_duration;
    uint8_t esc_hw4_current_thresold;
    uint16_t esc_hw4_current_max;
    float esc_hw4_divisor;
    float esc_hw4_current_multiplier;
    bool ibus_alternative_coordinates;
    uint8_t debug;
    bool esc_hw4_is_manual_offset;
    uint8_t analog_rate;
    bool xbus_use_alternative_volt_temp;
    uint8_t spare1;
    float esc_hw4_offset;
    uint32_t serial_monitor_baudrate;
    uint8_t serial_monitor_stop_bits;
    uint8_t serial_monitor_parity;
    uint16_t serial_monitor_timeout_ms;
    bool serial_monitor_inverted;
    int8_t airspeed_offset;
    int16_t airspeed_slope;
    float fuel_flow_ml_per_pulse;
    bool enable_fuel_flow;
    uint16_t xgzp68xxd_k;
    uint8_t enable_fuel_pressure;
    uint32_t spare3;
    uint32_t spare4;
    uint32_t spare5;
    uint32_t spare6;
    uint32_t spare7;
    uint32_t spare8;
    uint32_t spare9;
    uint32_t spare10;
    uint32_t spare11;
    uint32_t spare12;
    uint32_t spare13;
    uint32_t spare14;
    uint32_t spare15;
    uint32_t spare16;
    uint32_t spare17;
    uint32_t spare18;
    uint32_t spare19;
    uint32_t spare20;
} config_t;

typedef struct context_t {
    TaskHandle_t pwm_out_task_handle, uart0_notify_task_handle, uart1_notify_task_handle, uart_pio_notify_task_handle,
        receiver_task_handle, led_task_handle, usb_task_handle;
    QueueHandle_t uart0_queue_handle, uart1_queue_handle, uart_pio_queue_handle, tasks_queue_handle,
        sensors_queue_handle;
    alarm_pool_t *uart_alarm_pool;
    uint8_t debug, led_cycles;
    uint16_t led_cycle_duration;
} context_t;

float get_average(float alpha, float prev_value, float new_value);
float get_consumption(float current, uint16_t current_max, uint32_t *timestamp);
float voltage_read(uint8_t adc_num);
float get_altitude(float pressure, float temperature, float P0);

#endif