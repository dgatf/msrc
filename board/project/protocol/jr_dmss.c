#include "jr_dmss.h"

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
#include "esc_pwm.h"
#include "ms5611.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "uart.h"
#include "voltage.h"
#include "smart_esc.h"
#include "esc_omp_m4.h"
#include "esc_ztw.h"

#define JR_DMSS_TEMPERATURE_SENSOR_ID 1
#define JR_DMSS_RPM_SENSOR_ID 2
#define JR_DMSS_VARIO_SENSOR_ID 3
#define JR_DMSS_AIRSPEED_SENSOR_ID 5
#define JR_DMSS_BATTERY_SENSOR_ID 8

#define JR_DMSS_TEMPERATURE_TEMPERATURE_INDEX 0x0  // 1ºC
#define JR_DMSS_RPM_RPM_INDEX 0x0                  // 1 rpm
#define JR_DMSS_VARIO_ALTITUDE_INDEX 0x3           // 0.1m
#define JR_DMSS_VARIO_VSPEED_INDEX 0x2             // 0.1 m/s
#define JR_DMSS_VARIO_PRESSURE_INDEX 0x1           // 0.1 hPa
#define JR_DMSS_AIRSPEED_AIRSPEED_INDEX 0x0        // 1 km/h
#define JR_DMSS_BATTERY_VOLTAGE_INDEX 0x0          // 0.01 V
#define JR_DMSS_BATTERY_CURRENT_INDEX 0x80         // 0.01 A
#define JR_DMSS_BATTERY_CAPACITY_INDEX 0x8         // 1 mAh
#define JR_DMSS_BATTERY_POWER_INDEX 0x90           // 0.1 W

#define JR_DMSS_TIMEOUT_US 300
#define JR_DMSS_PACKET_LENGHT 1

#define PIN_BASE 12

typedef enum sensor_t {
    TEMPERATURE,
    RPM,
    ALTITUDE,
    VSPEED,
    PRESSURE,
    AIRSPEED,
    VOLTAGE,
    CURRENT,
    CAPACITY,
    POWER
} sensor_t;

static uint8_t s_crc_array[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21,
    0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c,
    0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c,
    0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66,
    0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4,
    0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6,
    0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e, 0xed,
    0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1,
    0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf,
    0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57,
    0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9,
    0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35};

static void process(float **sensor);
static void send_packet(uint8_t address, float **sensor);
static uint8_t crc_tab1e(uint8_t data, uint8_t crc);
static uint8_t crc8(uint8_t *buffer, uint8_t length);
static void set_config(float **sensor);

void jr_dmss_task(void *parameters) {
    float *sensor[10] = {NULL};
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(250000L, UART_RECEIVER_TX, UART_RECEIVER_RX, JR_DMSS_TIMEOUT_US, 8, 2, UART_PARITY_NONE, false, true);
    set_config(sensor);
    debug("\nJR Propo init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(sensor);
    }
}

static void process(float **sensor) {
    uint8_t len = uart0_available();
    uint8_t buffer[len];
    uart0_read_bytes(buffer, len);
    if (len == JR_DMSS_PACKET_LENGHT) {
        // debug("\nJR Propo (%u) < %X", uxTaskGetStackHighWaterMark(NULL), buffer[0]);
        send_packet(buffer[0], sensor);
    }
}

static void send_packet(uint8_t address, float **sensor) {
    uint8_t buffer[6];
    switch (address) {
        case JR_DMSS_TEMPERATURE_SENSOR_ID: {
            if (sensor[TEMPERATURE] == NULL) return;
            buffer[0] = JR_DMSS_TEMPERATURE_SENSOR_ID | 0xE0;
            buffer[1] = 0x3;
            buffer[2] = JR_DMSS_TEMPERATURE_TEMPERATURE_INDEX;
            uint16_t value = *sensor[TEMPERATURE];
            buffer[3] = value >> 8;
            buffer[4] = value;
            buffer[5] = crc8(buffer, 5);
            uart0_write_bytes(buffer, sizeof(buffer));
            vTaskResume(context.led_task_handle);
            debug("\nJR Propo (%u) > ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(buffer, sizeof(buffer), "0x%X ");
            break;
        }
        case JR_DMSS_RPM_SENSOR_ID: {
            if (sensor[RPM] == NULL) return;
            buffer[0] = JR_DMSS_RPM_SENSOR_ID | 0xE0;
            buffer[1] = 0x3;
            buffer[2] = JR_DMSS_RPM_RPM_INDEX;
            uint16_t value = *sensor[RPM];
            buffer[3] = value >> 8;
            buffer[4] = value;
            buffer[5] = crc8(buffer, 5);
            uart0_write_bytes(buffer, sizeof(buffer));
            vTaskResume(context.led_task_handle);
            debug("\nJR Propo (%u) > ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(buffer, sizeof(buffer), "0x%X ");
            break;
        }
        case JR_DMSS_VARIO_SENSOR_ID: {
            {
                static uint8_t index = 0;
                buffer[0] = JR_DMSS_VARIO_SENSOR_ID  | 0xE0;
                buffer[1] = 0x3;
                uint16_t value;
                if ((index % 3) == 0) {
                    if (sensor[ALTITUDE] == NULL) {
                        index++;
                        return;
                    }
                    buffer[2] = JR_DMSS_VARIO_ALTITUDE_INDEX;
                    value = *sensor[ALTITUDE] * 10;
                } else if ((index % 3) == 1) {
                    if (sensor[VSPEED] == NULL) {
                        index++;
                        return;
                    }
                    buffer[2] = JR_DMSS_VARIO_VSPEED_INDEX;
                    value = *sensor[VSPEED] * 10;
                } else if ((index % 3) == 2) {
                    if (sensor[PRESSURE] == NULL) {
                        index++;
                        return;
                    }
                    buffer[2] = JR_DMSS_VARIO_PRESSURE_INDEX;
                    value = *sensor[PRESSURE] / 100 * 10;  // Pa -> 0.1 hPa
                }
                buffer[3] = value >> 8;
                buffer[4] = value;
                buffer[5] = crc8(buffer, 5);
                uart0_write_bytes(buffer, sizeof(buffer));
                vTaskResume(context.led_task_handle);
                debug("\nJR Propo (%u) > ", uxTaskGetStackHighWaterMark(NULL));
                debug_buffer(buffer, sizeof(buffer), "0x%X ");
                index++;
                break;
            }
        }
        case JR_DMSS_AIRSPEED_SENSOR_ID: {
            if (sensor[AIRSPEED] == NULL) return;
            buffer[0] = JR_DMSS_AIRSPEED_SENSOR_ID | 0xE0;
            buffer[1] = 0x3;
            buffer[2] = JR_DMSS_AIRSPEED_AIRSPEED_INDEX;
            uint16_t value = *sensor[AIRSPEED];
            buffer[3] = value >> 8;
            buffer[4] = value;
            buffer[5] = crc8(buffer, 5);
            uart0_write_bytes(buffer, sizeof(buffer));
            vTaskResume(context.led_task_handle);
            debug("\nJR Propo (%u) > ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(buffer, sizeof(buffer), "0x%X ");
            break;
        }
        case JR_DMSS_BATTERY_SENSOR_ID: {
            static uint8_t index = 0;
            buffer[0] = JR_DMSS_BATTERY_SENSOR_ID | 0xE0;
            buffer[1] = 0x3;
            uint16_t value;
            if ((index % 3) == 0) {
                if (sensor[VOLTAGE] == NULL) {
                    index++;
                    return;
                }
                buffer[2] = JR_DMSS_BATTERY_VOLTAGE_INDEX;
                value = *sensor[VOLTAGE] * 100;
            } else if ((index % 3) == 1) {
                if (sensor[CURRENT] == NULL) {
                    index++;
                    return;
                }
                buffer[2] = JR_DMSS_BATTERY_CURRENT_INDEX;
                value = *sensor[CURRENT] * 100;
            } else if ((index % 3) == 2) {
                if (sensor[CAPACITY] == NULL) {
                    index++;
                    return;
                }
                buffer[2] = JR_DMSS_BATTERY_CAPACITY_INDEX;
                value = *sensor[CAPACITY];
            }
            buffer[3] = value >> 8;
            buffer[4] = value;
            buffer[5] = crc8(buffer, 5);
            uart0_write_bytes(buffer, sizeof(buffer));
            vTaskResume(context.led_task_handle);
            debug("\nJR Propo (%u) > ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(buffer, sizeof(buffer), "0x%X ");
            index++;
            break;
        }
    }
}

static uint8_t crc_tab1e(uint8_t data, uint8_t crc) {
    uint16_t index = (data ^ crc) & 0xff;
    crc = s_crc_array[index];
    return crc;
}

static uint8_t crc8(uint8_t *buffer, uint8_t length) {
    uint8_t crc = 0;
    while (length-- > 0) {
        crc = crc_tab1e(*buffer++, crc);
    }
    return crc;
}

static void set_config(float **sensor) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[VOLTAGE] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW4) {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier,
                                          config->enable_pwm_out,
                                          config->enable_esc_hw4_init_delay,
                                          config->alpha_rpm,
                                          config->alpha_voltage,
                                          config->alpha_current,
                                          config->alpha_temperature,
                                          config->esc_hw4_divisor,
                                          config->esc_hw4_current_multiplier,
                                          config->esc_hw4_current_thresold,
                                          config->esc_hw4_current_max,
                                          config->esc_hw4_is_manual_offset,
                                          config->esc_hw4_offset,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_hw4_task", STACK_ESC_HW4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_pwm_out) {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            context.pwm_out_task_handle = task_handle;
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature_fet;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
    }
    if (config->esc_protocol == ESC_HW5) {
        esc_hw5_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw5_task, "esc_hw5_task", STACK_ESC_HW5, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature_fet;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
    }
    if (config->esc_protocol == ESC_CASTLE) {
        esc_castle_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                             config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_KONTRONIK) {
        esc_kontronik_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage,  config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_kontronik_task, "esc_kontronik_task", STACK_ESC_KONTRONIK, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature_fet;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_F) {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                            config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_HV) {
        esc_apd_hv_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_hv_task, "esc_apd_hv_task", STACK_ESC_APD_HV, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_SMART) {
        smart_esc_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temperature_fet = malloc(sizeof(float));
        parameter.temperature_bec = malloc(sizeof(float));
        parameter.voltage_bec = malloc(sizeof(float));
        parameter.current_bec = malloc(sizeof(float));
        parameter.temperature_bat = malloc(sizeof(float));
        parameter.current_bat = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        for (uint i = 0; i < 18; i++) parameter.cell[i] = malloc(sizeof(float));
        parameter.cells = malloc(sizeof(uint8_t));
        parameter.cycles = malloc(sizeof(uint16_t));
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temperature_fet;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_OMP_M4) {
        esc_omp_m4_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_omp_m4_task, "esc_omp_m4_task", STACK_ESC_OMP_M4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temp_esc;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_ZTW) {
        esc_ztw_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.bec_voltage = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_ztw_task, "esc_ztw_task", STACK_ESC_ZTW, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.rpm;
        sensor[RPM] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.temp_esc;
        sensor[TEMPERATURE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.voltage;
        sensor[VOLTAGE] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current) {
        current_parameters_t parameter = {1,
                                          config->analog_rate,
                                          config->alpha_current,
                                          config->analog_current_multiplier,
                                          config->analog_current_offset,
                                          config->analog_current_autoffset,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.current;
        sensor[CURRENT] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.consumption;
        sensor[CAPACITY] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.ntc;
        sensor[TEMPERATURE] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.altitude;
        sensor[ALTITUDE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.vspeed;
        sensor[VSPEED] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.pressure;
        sensor[PRESSURE] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.altitude;
        sensor[ALTITUDE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.vspeed;
        sensor[VSPEED] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.pressure;
        sensor[PRESSURE] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.altitude;
        sensor[ALTITUDE] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.vspeed;
        sensor[VSPEED] = new_sensor;
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.pressure;
        sensor[PRESSURE] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 100,
                                           (float)config->airspeed_vcc / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(float));
        new_sensor = parameter.airspeed;
        sensor[AIRSPEED] = new_sensor;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
