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

#define FRAME_LENGHT 10

typedef struct fbus_packet_t {
    uint8_t len;
    uint8_t sensor_id;
    uint8_t frame_id;
    uint16_t data_id;
    uint32_t value;
    uint8_t crc;
} __attribute__((packed)) fbus_packet_t;

static SemaphoreHandle_t semaphore_sensor = NULL;
static uint8_t sensor_id;

static void process(smartport_parameters_t *parameter);
static void sensor_task(void *parameters);
static void sensor_void_task(void *parameters);
static void sensor_double_task(void *parameters);
static void sensor_coordinates_task(void *parameters);
static void sensor_datetime_task(void *parameters);
static void sensor_cell_task(void *parameters);
static void sensor_cell_individual_task(void *parameters);
static void sensor_gpio_task(void *parameters);
static void send_packet(uint8_t frame_id, uint16_t data_id, uint32_t value);
static void set_config(smartport_parameters_t *parameter);

void fbus_task(void *parameters) {
    smartport_parameters_t parameter;
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(460800, UART_RECEIVER_TX, UART_RECEIVER_RX, TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    semaphore_sensor = xSemaphoreCreateBinary();
    xSemaphoreTake(semaphore_sensor, 0);
    set_config(&parameter);
    debug("\nFbus init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void sensor_task(void *parameters) {
    smartport_sensor_parameters_t parameter = *(smartport_sensor_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        int32_t data_formatted = smartport_format(parameter.data_id, *parameter.value);
        debug("\nFBUS. Sensor (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, parameter.data_id, data_formatted);
    }
}

static void sensor_gpio_task(void *parameters) {
    smartport_sensor_gpio_parameters_t parameter = *(smartport_sensor_gpio_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    uint cont = 0;
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (parameter.gpio_mask) {
            while (!(parameter.gpio_mask & (1 << cont))) {
                cont++;
                if (cont == 6) cont = 0;
            }
            float value = *parameter.value & (1 << cont) ? 1 : 0;
            uint16_t data_id = parameter.data_id + 17 + cont;
            int32_t data_formatted = smartport_format(data_id, value);
            debug("\nFBUS. Sensor GPIO (%u) > GPIO: %u STATE: %u > ", uxTaskGetStackHighWaterMark(NULL), 17 + cont,
                  (uint)value);
            send_packet(0x10, data_id, data_formatted);
            cont++;
            if (cont == 6) cont = 0;
        }
    }
}

static void sensor_void_task(void *parameters) {
    while (1) {
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (context.debug == 2) printf("\nFBUS. Sensor void (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0, 0, 0);
    }
}

static void sensor_double_task(void *parameters) {
    smartport_sensor_double_parameters_t parameter = *(smartport_sensor_double_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        float v_l = parameter.value_l ? *parameter.value_l : 0.0f;
        float v_h = parameter.value_h ? *parameter.value_h : 0.0f;
        uint32_t data_formatted = smartport_format_double(parameter.data_id, v_l, v_h);
        debug("\nFBUS. Sensor double (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, parameter.data_id, data_formatted);
    }
}

static void sensor_coordinates_task(void *parameters) {
    smartport_sensor_coordinate_parameters_t parameter = *(smartport_sensor_coordinate_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted;
        if (parameter.type == SMARTPORT_LATITUDE)
            data_formatted = smartport_format_coordinate(parameter.type, *parameter.latitude);
        else
            data_formatted = smartport_format_coordinate(parameter.type, *parameter.longitude);
        parameter.type = !parameter.type;
        debug("\nFBUS. Sensor coordinates (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, GPS_LONG_LATI_FIRST_ID, data_formatted);
    }
}

static void sensor_datetime_task(void *parameters) {
    smartport_sensor_datetime_parameters_t parameter = *(smartport_sensor_datetime_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted;
        if (parameter.type == SMARTPORT_DATE)
            data_formatted = smartport_format_datetime(parameter.type, *parameter.date);
        else
            data_formatted = smartport_format_datetime(parameter.type, *parameter.time);
        parameter.type = !parameter.type;
        debug("\nFBUS. Sensor datetime (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, GPS_TIME_DATE_FIRST_ID, data_formatted);
    }
}

static void sensor_cell_task(void *parameters) {
    smartport_sensor_cell_parameters_t parameter = *(smartport_sensor_cell_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    uint8_t cell_index = 0;
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (!*parameter.cell_count) return;
        uint32_t data_formatted = smartport_format_cell(cell_index, *parameter.cell_voltage);
        cell_index++;
        if (cell_index > *parameter.cell_count - 1) cell_index = 0;
        debug("\nFBUS. Sensor cell (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, CELLS_FIRST_ID, data_formatted);
    }
}

static void sensor_cell_individual_task(void *parameters) {
    smartport_sensor_cell_individual_parameters_t parameter =
        *(smartport_sensor_cell_individual_parameters_t *)parameters;

    xTaskNotifyGive(context.receiver_task_handle);
    uint8_t cell_index = 0;

    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);

        // No cells configured → skip
        if (!parameter.cell_count || *parameter.cell_count == 0) {
            continue;
        }

        float value = 0.0f;

        // Safety: check index and pointer before dereferencing
        if (cell_index < *parameter.cell_count && parameter.cell_voltage[cell_index] != NULL) {
            value = *parameter.cell_voltage[cell_index];
        }

        uint32_t data_formatted = smartport_format_cell(cell_index, value);

        debug("\nFBUS. Sensor cell (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, CELLS_FIRST_ID, data_formatted);

        // Next cell
        cell_index++;
        if (cell_index >= *parameter.cell_count) {
            cell_index = 0;
        }
    }
}

static void process(smartport_parameters_t *parameter) {
    uint lenght = uart0_available();
    if (lenght > 128 || lenght < 10) return;
    if (lenght) {
        uint8_t data[lenght];
        uart0_read_bytes(data, lenght);
        uint8_t crc = smartport_get_crc(data, data[0] + 1);  // crc from len, size len + 1
        if (crc != data[data[0] + 1]) {
            debug("\nFBUS. Bad control CRC 0x%X - 0x%X", data[data[0] + 1], crc);
            return;
        }
        if (data[1] == 0xFF) memmove(data, data + data[0] + 2, FRAME_LENGHT);  // len not include len and crc
        debug("\nFBUS (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(data, FRAME_LENGHT, "0x%X ");
        crc = smartport_get_crc(data, data[0] + 1);  // crc from len, size len + 1 = 9
        if (crc != data[data[0] + 1]) {
            debug("\nFBUS. Bad downlink CRC 0x%X - 0x%X", data[data[0] + 1], crc);
            return;
        }
        // send telemetry
        uint16_t data_id = ((uint16_t)data[4] << 8) | data[3];
        if (data[0] == 0x08 && data[1] == smartport_sensor_id_to_crc(sensor_id) &&
            (data[2] == 0x00 || data[2] == 0x10)) {
            xSemaphoreGive(semaphore_sensor);
            vTaskDelay(4 / portTICK_PERIOD_MS);
            xSemaphoreTake(semaphore_sensor, 0);
        }
        // receive & send packet
        else if (data[0] == 0x08 && (data_id == parameter->data_id || data_id == 0xFFFF)) {
            uint8_t frame_id = data[2];
            uint value = (uint32_t)data[8] << 24 | (uint32_t)data[7] << 16 | (uint16_t)data[6] << 8 | data[5];
            debug("\nFBUS. Received packet (%u) FrameId 0x%X DataId 0x%X Value 0x%X < ",
                  uxTaskGetStackHighWaterMark(NULL), frame_id, data_id, value);
            debug_buffer(data, 10, "0x%X ");
            smartport_packet_t packet = smartport_process_packet(parameter, frame_id, data_id, value);
            if (packet.data_id != 0) {
                send_packet(packet.frame_id, packet.data_id, packet.value);
                debug("\nFBUS. Send packet (%u) FrameId 0x%X DataId 0x%X Value 0x%X", uxTaskGetStackHighWaterMark(NULL),
                      packet.frame_id, packet.data_id, packet.value);
            }
        }
    }
}

static void send_packet(uint8_t frame_id, uint16_t data_id, uint32_t value) {
    fbus_packet_t packet;
    packet.len = 8;
    packet.sensor_id = smartport_sensor_id_to_crc(sensor_id);
    packet.frame_id = frame_id;
    packet.data_id = data_id;
    packet.value = value;
    packet.crc = smartport_get_crc(((uint8_t *)&packet), FRAME_LENGHT - 1);
    uart0_write_bytes((uint8_t *)&packet, sizeof(packet));
    debug_buffer((uint8_t *)&packet, sizeof(packet), "0x%X ");
    // blink
    vTaskResume(context.led_task_handle);
}

static void set_config(smartport_parameters_t *parameter) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *baro_temp = NULL, *baro_pressure = NULL;
    parameter->sensor_id = config->smartport_sensor_id;
    sensor_id = config->smartport_sensor_id;
    parameter->data_id = 0x5100;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = NULL;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = NULL;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
                                          config->esc_hw4_voltage_multiplier,
                                          config->esc_hw4_current_multiplier,
                                          config->esc_hw4_current_thresold,
                                          config->esc_hw4_current_max,
                                          config->esc_hw4_is_manual_offset,
                                          config->esc_hw4_auto_detect,
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

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temperature_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID + 1;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID + 1;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_SMART) {
        smart_esc_parameters_t parameter;
        parameter.calc_consumption = config->smart_esc_calc_consumption;
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
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 4, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_individual_parameters_t parameter_sensor_cell;
        // rpm & consumption
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // voltage & current
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // bec. voltage & current
        parameter_sensor_double.data_id = SBEC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // temp_fet
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // temp_bec
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // temp_bat
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temperature_bat;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // current_bat
        parameter_sensor.data_id = CURR_FIRST_ID + 1;
        parameter_sensor.value = parameter.current_bat;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // cells
        parameter_sensor_cell.cell_count = parameter.cells;  // Pointer provided by ESC_SMART task
        for (uint i = 0; i < 18; i++) {
            parameter_sensor_cell.cell_voltage[i] = parameter.cell[i];  // One pointer per cell
        }
        parameter_sensor_cell.rate = config->refresh_rate_voltage;

        xTaskCreate(sensor_cell_individual_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL,
                    (void *)&parameter_sensor_cell, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // cycles
        /*parameter_sensor.data_id = DIY_FIRST_ID + 100;
        parameter_sensor.value = parameter.cycles;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_cell_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);*/
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temp_esc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temp_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temp_esc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temp_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_OPENYGE) {
        esc_openyge_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.pwm_out = config->enable_pwm_out;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temperature_fet = malloc(sizeof(float));
        parameter.temperature_bec = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.voltage_bec = malloc(sizeof(float));
        parameter.current_bec = malloc(sizeof(float));
        parameter.throttle = malloc(sizeof(float));
        parameter.pwm_percent = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_openyge_task, "esc_openyge_task", STACK_ESC_OPENYGE, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;

        // RPM and Consumption sensor
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Main voltage and current sensor
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // FET Temperature sensor
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // BEC Temperature sensor
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // BEC voltage and current sensor
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID + 2;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Cell voltage sensor
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps) {
        gps_parameters_t parameter;
        parameter.protocol = config->gps_protocol;
        parameter.baudrate = config->gps_baudrate;
        parameter.rate = config->gps_rate;
        parameter.lat = malloc(sizeof(double));
        parameter.lon = malloc(sizeof(double));
        parameter.alt = malloc(sizeof(float));
        parameter.spd = malloc(sizeof(float));
        parameter.cog = malloc(sizeof(float));
        parameter.hdop = malloc(sizeof(float));
        parameter.sat = malloc(sizeof(float));
        parameter.time = malloc(sizeof(float));
        parameter.date = malloc(sizeof(float));
        parameter.vspeed = malloc(sizeof(float));
        parameter.dist = malloc(sizeof(float));
        parameter.spd_kmh = malloc(sizeof(float));
        parameter.fix = malloc(sizeof(float));
        parameter.vdop = malloc(sizeof(float));
        parameter.speed_acc = malloc(sizeof(float));
        parameter.h_acc = malloc(sizeof(float));
        parameter.v_acc = malloc(sizeof(float));
        parameter.track_acc = malloc(sizeof(float));
        parameter.n_vel = malloc(sizeof(float));
        parameter.e_vel = malloc(sizeof(float));
        parameter.v_vel = malloc(sizeof(float));
        parameter.alt_elipsiod = malloc(sizeof(float));
        parameter.pdop = malloc(sizeof(float));
        xTaskCreate(gps_task, "gps_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_coordinate_parameters_t parameter_sensor_coordinate;
        parameter_sensor_coordinate.type = SMARTPORT_LATITUDE;
        parameter_sensor_coordinate.latitude = parameter.lat;
        parameter_sensor_coordinate.longitude = parameter.lon;
        parameter_sensor_coordinate.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_coordinates_task, "sensor_coordinates_task", STACK_SENSOR_SMARTPORT,
                    (void *)&parameter_sensor_coordinate, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_datetime_parameters_t parameter_sensor_datetime;
        parameter_sensor_datetime.type = SMARTPORT_DATE;
        parameter_sensor_datetime.date = parameter.date;
        parameter_sensor_datetime.time = parameter.time;
        parameter_sensor_datetime.rate = 1000;
        xTaskCreate(sensor_datetime_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor_datetime, 3,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = GPS_ALT_FIRST_ID;
        parameter_sensor.value = parameter.alt;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GPS_SPEED_FIRST_ID;
        parameter_sensor.value = parameter.spd;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GPS_COURS_FIRST_ID;
        parameter_sensor.value = parameter.cog;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID + 1;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 3;
        parameter_sensor.value = parameter.sat;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 4;
        parameter_sensor.value = parameter.dist;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 5;
        parameter_sensor.value = parameter.pdop;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = A3_FIRST_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = CURR_FIRST_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = NULL;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_current;
        xTaskCreate(sensor_double_task, "sensor_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.ntc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 1000,
                                           (float)config->airspeed_vcc / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = AIR_SPEED_FIRST_ID;
        parameter_sensor.value = parameter.airspeed;
        parameter_sensor.rate = config->refresh_rate_airspeed;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = GASSUIT_FLOW_FIRST_ID;
        parameter_sensor.value = parameter.consumption_instant;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GASSUIT_RES_VOL_FIRST_ID;
        parameter_sensor.value = parameter.consumption_total;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->gpio_mask) {
        gpio_parameters_t parameter = {config->gpio_mask, config->gpio_interval, malloc(sizeof(float))};
        xTaskCreate(gpio_task, "gpio_task", STACK_GPIO, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_gpio_parameters_t parameter_sensor;
        parameter_sensor.data_id = DIY_FIRST_ID;
        parameter_sensor.value = parameter.value;
        parameter_sensor.rate = config->gpio_interval;
        parameter_sensor.gpio_mask = config->gpio_mask;
        xTaskCreate(sensor_gpio_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gyro) {
        mpu6050_parameters_t parameter = {1,
                                          0,
                                          config->mpu6050_acc_scale,
                                          config->mpu6050_gyro_scale,
                                          config->mpu6050_gyro_weighting,
                                          config->mpu6050_filter,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float))};
        xTaskCreate(mpu6050_task, "mpu6050_task", STACK_MPU6050, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = DIY_FIRST_ID + 6;
        parameter_sensor.value = parameter.pitch;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 7;
        parameter_sensor.value = parameter.roll;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 8;
        parameter_sensor.value = parameter.yaw;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ACCX_FIRST_ID;
        parameter_sensor.value = parameter.acc_x;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ACCY_FIRST_ID;
        parameter_sensor.value = parameter.acc_y;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ACCZ_FIRST_ID;
        parameter_sensor.value = parameter.acc_z;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 9;
        parameter_sensor.value = parameter.acc;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_lipo && config->lipo_cells > 0) {
        smartport_sensor_cell_individual_parameters_t parameter_sensor_cell;
        float *cell_prev = NULL;

        // Maximum supported cells: 6 (two INA3221 devices)
        uint8_t lipo_cells = MIN(config->lipo_cells, 6);

        // Initialize all cell pointers to NULL (safety)
        for (uint i = 0; i < 18; i++) {
            parameter_sensor_cell.cell_voltage[i] = NULL;
        }

        // Configure task parameters
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        parameter_sensor_cell.cell_count = malloc(sizeof(uint8_t));
        *parameter_sensor_cell.cell_count = lipo_cells;

        // --- First INA3221: cells 0–2 -----------------------------------------
        uint8_t cells_first = MIN(lipo_cells, 3);

        if (cells_first > 0) {
            ina3221_parameters_t parameter = {
                .i2c_address = 0x40,
                .filter = config->ina3221_filter,
                .cell_count = cells_first,
                .cell[0] = malloc(sizeof(float)),
                .cell[1] = malloc(sizeof(float)),
                .cell[2] = malloc(sizeof(float)),
                .cell_prev = malloc(sizeof(float)),
            };

            // First INA has no previous cell reference
            *parameter.cell_prev = 0;
            cell_prev = parameter.cell[2];

            xTaskCreate(ina3221_task, "ina3221_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Store cell pointers for SmartPort
            for (uint i = 0; i < cells_first; i++) {
                parameter_sensor_cell.cell_voltage[i] = parameter.cell[i];
            }
        }

        // --- Second INA3221: cells 3–5 ----------------------------------------
        if (lipo_cells > 3) {
            uint8_t cells_second = MIN((uint8_t)(lipo_cells - 3), (uint8_t)3);

            ina3221_parameters_t parameter = {
                .i2c_address = 0x41,
                .filter = config->ina3221_filter,
                .cell_count = cells_second,
                .cell[0] = malloc(sizeof(float)),
                .cell[1] = malloc(sizeof(float)),
                .cell[2] = malloc(sizeof(float)),
                .cell_prev = cell_prev,  // Link to the last cell of the previous INA
            };

            xTaskCreate(ina3221_task, "ina3221_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Store cell pointers for SmartPort
            for (uint i = 0; i < cells_second; i++) {
                parameter_sensor_cell.cell_voltage[i + 3] = parameter.cell[i];
            }
        }

        // --- Start SmartPort task: cycle through individual cells -------------
        xTaskCreate(sensor_cell_individual_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL,
                    (void *)&parameter_sensor_cell, 3, &task_handle);

        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
