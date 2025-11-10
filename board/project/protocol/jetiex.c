#include "jetiex.h"

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
#include "ina3221.h"
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

#define JETIEX_WAIT 0
#define JETIEX_SEND 1

#define JETIEX_PACKET_LENGHT 8
#define JETIEX_TIMEOUT_US 500
#define JETIEX_BAUDRATE_TIMEOUT_MS 5000

static void process(uint *baudrate, sensor_jetiex_t **sensor);
static void send_packet(uint8_t packet_id, sensor_jetiex_t **sensor);
static void add_sensor_text(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor);
static void add_sensor_value(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor);
static int64_t timeout_callback(alarm_id_t id, void *parameters);
static uint8_t crc8(uint8_t *crc, uint8_t crc_length);
static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
static uint16_t crc16(uint8_t *p, uint16_t len);
static uint16_t update_crc16(uint16_t crc, uint8_t data);

void jetiex_task(void *parameters) {
    uint baudrate = 125000L;
    sensor_jetiex_t *sensor[16] = {NULL};
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(baudrate, UART_RECEIVER_TX, UART_RECEIVER_RX, JETIEX_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    jeti_set_config(sensor);
    debug("\nJeti Ex init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&baudrate, sensor);
    }
}

uint8_t jeti_create_telemetry_buffer(uint8_t *buffer, bool packet_type, sensor_jetiex_t **sensor) {
    static uint8_t sensor_index_value = 0;
    static int8_t sensor_index_text = -1;
    uint8_t buffer_index = 7;
    if (sensor[0] == NULL) return 0;
    if (packet_type) {
        buffer[1] = 0x40;

        add_sensor_value(buffer, &buffer_index, sensor_index_value + 1, sensor[sensor_index_value]);
        sensor_index_value++;
        if (sensor_index_value >= 16 || sensor[sensor_index_value] == NULL)
            sensor_index_value = 0;
        else {
            add_sensor_value(buffer, &buffer_index, sensor_index_value + 1, sensor[sensor_index_value]);
            sensor_index_value++;
            if (sensor_index_value >= 16 || sensor[sensor_index_value] == NULL) sensor_index_value = 0;
        }
    } else {
        add_sensor_text(buffer, &buffer_index, sensor_index_text + 1,
                        sensor_index_text == -1 ? NULL : sensor[sensor_index_text]);
        sensor_index_text++;
        if (sensor_index_text >= 16 || sensor[sensor_index_text] == NULL) sensor_index_text = -1;
    }
    buffer[0] = 0x0F;
    buffer[1] |= buffer_index - 1;
    buffer[2] = JETIEX_MFG_ID_LOW;
    buffer[3] = JETIEX_MFG_ID_HIGH;
    buffer[4] = JETIEX_DEV_ID_LOW;
    buffer[5] = JETIEX_DEV_ID_HIGH;
    buffer[6] = 0x00;
    buffer[buffer_index] = crc8(buffer + 1, buffer_index - 1);
    return buffer_index + 1;
}

void jeti_add_sensor(sensor_jetiex_t *new_sensor, sensor_jetiex_t **sensors) {
    static uint8_t sensor_count = 0;
    if (sensor_count < 15) {
        sensors[sensor_count] = new_sensor;
        new_sensor->data_id = sensor_count;
        sensor_count++;
    }
}

void jeti_set_config(sensor_jetiex_t **sensor) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_jetiex_t *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;

    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
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

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp FET",
                                        "C", parameter.temperature_fet};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp BEC",
                                        "C", parameter.temperature_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp FET",
                                        "C", parameter.temperature_fet};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp BEC",
                                        "C", parameter.temperature_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,          JETIEX_FORMAT_0_DECIMAL, "Temp Motor",
                                        "C", parameter.temperature_motor};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Voltage BEC", "C", parameter.voltage_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current BEC", "C", parameter.current_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
    }
    if (config->esc_protocol == ESC_CASTLE) {
        esc_castle_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                             config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temperature", "C", parameter.temperature};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,       JETIEX_FORMAT_2_DECIMAL, "Ripple Voltage BEC",
                                        "V", parameter.ripple_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "BEC Voltage", "V", parameter.voltage_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "BEC Current", "A", parameter.current_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Current BEC", "A", parameter.current_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage BEC", "V", parameter.voltage_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp FET",
                                        "C", parameter.temperature_fet};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp BEC",
                                        "C", parameter.temperature_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp", "C", parameter.temperature};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp", "C", parameter.temperature};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp ESC", "C", parameter.temp_esc};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp Motor", "C", parameter.temp_motor};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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
        xTaskCreate(esc_omp_m4_task, "esc_ztw_task", STACK_ESC_ZTW, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage BEC", "V", parameter.bec_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp ESC", "C", parameter.temp_esc};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp Motor", "C", parameter.temp_motor};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,     JETIEX_FORMAT_2_DECIMAL, "Cell Voltage",
                                        "V", parameter.cell_voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 4, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current BEC", "A", parameter.current_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage BEC", "V", parameter.voltage_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp FET",
                                        "C", parameter.temperature_fet};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,        JETIEX_FORMAT_0_DECIMAL, "Temp BEC",
                                        "C", parameter.temperature_bec};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT6, JETIEX_FORMAT_0_DECIMAL, "Sats", "", parameter.sat};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_COORDINATES, JETIEX_FORMAT_LAT, "Latitude", "", parameter.lat};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_COORDINATES, JETIEX_FORMAT_LON, "Longitude", "", parameter.lon};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_1_DECIMAL, "Altitude", "m", parameter.alt};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        if (config->jeti_gps_speed_units_kmh)
            *new_sensor =
                (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Speed", "km/h", parameter.spd_kmh};
        else
            *new_sensor =
                (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Speed", "kts", parameter.spd};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "COG", "", parameter.cog};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Dist", "m", parameter.dist};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_TIMEDATE, JETIEX_FORMAT_TIME, "Time", "", parameter.time};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_TIMEDATE, JETIEX_FORMAT_DATE, "Date", "", parameter.date};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "HDOP", "", parameter.hdop};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "PDOP", "", parameter.pdop};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,     JETIEX_TYPE_INT22,    JETIEX_FORMAT_0_DECIMAL, "Consumption",
                                        "mAh", parameter.consumption};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temperature", "C", parameter.ntc};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,    JETIEX_FORMAT_0_DECIMAL, "Air temperature",
                                        "C", parameter.temperature};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Altitude", "m", parameter.altitude};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_2_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,    JETIEX_FORMAT_0_DECIMAL, "Air temperature",
                                        "C", parameter.temperature};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Altitude", "m", parameter.altitude};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_2_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,   JETIEX_TYPE_INT14,    JETIEX_FORMAT_0_DECIMAL, "Air temperature",
                                        "C", parameter.temperature};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Altitude", "m", parameter.altitude};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_2_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Air speed", "km/h", parameter.airspeed};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,
                                        JETIEX_TYPE_INT14,
                                        JETIEX_FORMAT_2_DECIMAL,
                                        "Instant consumption",
                                        "ml/min",
                                        parameter.consumption_instant};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0,    JETIEX_TYPE_INT14,          JETIEX_FORMAT_1_DECIMAL, "Total consumption",
                                        "ml", parameter.consumption_total};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_pressure) {
        xgzp68xxd_parameters_t parameter = {config->xgzp68xxd_k, malloc(sizeof(float)), malloc(sizeof(float))};

        xTaskCreate(xgzp68xxd_task, "fuel_pressure_task", STACK_FUEL_PRESSURE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor =
            (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Tank pressure", "Pa", parameter.pressure};
        jeti_add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Pitch", "dps", parameter.pitch};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Roll", "dps", parameter.roll};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Yaw", "dps", parameter.yaw};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Acc X", "g", parameter.acc_x};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Acc Y", "g", parameter.acc_y};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Acc Z", "g", parameter.acc_z};
        jeti_add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Acc", "g", parameter.acc};
        jeti_add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_lipo) {
        ina3221_parameters_t parameter = {
            .filter = config->ina3221_filter,
            .cell_count = config->lipo_cells,
            .cell[0] = malloc(sizeof(float)),
            .cell[1] = malloc(sizeof(float)),
            .cell[2] = malloc(sizeof(float)),
        };
        xTaskCreate(ina3221_task, "ina3221_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        for (uint8_t i = 0; i < parameter.cell_count; i++) {
            new_sensor = malloc(sizeof(sensor_jetiex_t));
            new_sensor->data_id = 0;
            new_sensor->type = JETIEX_TYPE_INT14;
            new_sensor->format = JETIEX_FORMAT_2_DECIMAL;
            sprintf(new_sensor->text, "Cell %d", i + 1);
            jeti_add_sensor(new_sensor, sensor);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
}

static void process(uint *baudrate, sensor_jetiex_t **sensor) {
    static alarm_id_t timeout_alarm_id = 0;
    uint8_t packetId;
    uint8_t length = uart0_available();
    if (length) {
        uint8_t data[length];
        uart0_read_bytes(data, length);
        if (context.debug == 2) {
            printf("\nJeti Ex(%u) < ", uxTaskGetStackHighWaterMark(NULL));
            for (uint8_t i = 0; i < length; i++) {
                printf("%X ", data[i]);
            }
        }
        uint8_t packet[JETIEX_PACKET_LENGHT];
        if (data[0] == 0x3E && data[1] == 0x3 && length - data[2] == JETIEX_PACKET_LENGHT) {
            memcpy(packet, data + data[2], JETIEX_PACKET_LENGHT);
            debug("\nJeti Ex(%u) < ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(packet, JETIEX_PACKET_LENGHT, "0x%X ");
        } else if (length == JETIEX_PACKET_LENGHT) {
            memcpy(packet, data, JETIEX_PACKET_LENGHT);
        } else {
            return;
        }
        if (crc16(packet, JETIEX_PACKET_LENGHT) == 0) {
            if (packet[0] == 0x3D && packet[1] == 0x01 && packet[4] == 0x3A) {
                if (timeout_alarm_id) cancel_alarm(timeout_alarm_id);
                uint8_t packet_id = packet[3];
                send_packet(packet_id, sensor);
                timeout_alarm_id = add_alarm_in_ms(JETIEX_BAUDRATE_TIMEOUT_MS, timeout_callback, &baudrate, false);
            }
        }
    }
}

static void send_packet(uint8_t packet_id, sensor_jetiex_t **sensor) {
    static uint8_t packet_count = 0;
    uint8_t ex_buffer[36] = {0};
    uint8_t length_telemetry_buffer = jeti_create_telemetry_buffer(ex_buffer + 6, packet_count % 16, sensor);
    ex_buffer[0] = 0x3B;
    ex_buffer[1] = 0x01;
    ex_buffer[2] = length_telemetry_buffer + 8;
    ex_buffer[3] = packet_id;
    ex_buffer[4] = 0x3A;
    ex_buffer[5] = length_telemetry_buffer;
    uint16_t crc = crc16(ex_buffer, length_telemetry_buffer + 6);
    ex_buffer[length_telemetry_buffer + 6] = crc;
    ex_buffer[length_telemetry_buffer + 7] = crc >> 8;
    uart0_write_bytes(ex_buffer, length_telemetry_buffer + 8);
    debug("\nJeti Ex %s (%u) > ", packet_count % 16 ? "Values " : "Text ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(ex_buffer, length_telemetry_buffer + 8, "0x%X ");
    packet_count++;

    // blink led
    vTaskResume(context.led_task_handle);
}

static void add_sensor_value(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor) {
    if (sensor) {
        uint8_t format = sensor->format << 5;
        if (sensor->type == JETIEX_TYPE_INT6) {
            int8_t value = *sensor->value * pow(10, sensor->format);
            if (value > 0x1F)
                value = 0x1F;
            else if (value < -0x1F)
                value = -0x1F;
            value &= ~(3 << 5);
            value |= format;
            *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
            *(buffer + *buffer_index + 1) = value;
            *buffer_index += 2;
        } else if (sensor->type == JETIEX_TYPE_INT14) {
            int16_t value = *sensor->value * pow(10, sensor->format);
            if (value > 0x1FFF) value = 0x1FFF;
            if (value < -0x1FFF) value = -0x1FFF;
            value &= ~((uint16_t)3 << (5 + 8));
            value |= (uint16_t)format << 8;
            *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
            *(buffer + *buffer_index + 1) = value;
            *(buffer + *buffer_index + 2) = value >> 8;
            *buffer_index += 3;
        } else if (sensor->type == JETIEX_TYPE_INT22) {
            int32_t value = *sensor->value * pow(10, sensor->format);
            if (value > 0x1FFFFF)
                value = 0x1FFFFF;
            else if (value < -0x1FFFFF)
                value = -0x1FFFFF;
            value &= ~((uint32_t)3 << (5 + 16));
            value |= (uint32_t)format << 16;
            *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
            *(buffer + *buffer_index + 1) = value;
            *(buffer + *buffer_index + 2) = value >> 8;
            *(buffer + *buffer_index + 3) = value >> 16;
            *buffer_index += 4;
        } else if (sensor->type == JETIEX_TYPE_INT30) {
            int32_t value = *sensor->value * pow(10, sensor->format);
            if (value > 0x1FFFFFFF)
                value = 0x1FFFFFFF;
            else if (value < -0x1FFFFFFF)
                value = -0x1FFFFFFF;
            value &= ~((uint32_t)3 << (5 + 24));
            value |= (uint32_t)format << 24;
            *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
            *(buffer + *buffer_index + 1) = value;
            *(buffer + *buffer_index + 2) = value >> 8;
            *(buffer + *buffer_index + 3) = value >> 16;
            *(buffer + *buffer_index + 4) = value >> 24;
            *buffer_index += 5;
        } else if (sensor->type == JETIEX_TYPE_TIMEDATE) {
            // rawvalue: yymmdd/hhmmss
            // byte 1: day/second
            // byte 2: month/minute
            // byte 3(bits 1-5): year/hour
            // byte 3(bit 6): 0=time 1=date
            uint32_t value = *sensor->value;
            uint8_t hourYearFormat = format;
            hourYearFormat |= value / 10000;                              // hour, year
            uint8_t minuteMonth = (value / 100 - (value / 10000) * 100);  // minute, month
            uint8_t secondDay = value - (value / 100) * 100;              // second, day
            *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
            *(buffer + *buffer_index + 1) = secondDay;
            *(buffer + *buffer_index + 2) = minuteMonth;
            *(buffer + *buffer_index + 3) = hourYearFormat;
            *buffer_index += 4;
        } else if (sensor->type == JETIEX_TYPE_COORDINATES) {
            // rawvalue: minutes
            // byte 1-2: MMmmm
            // byte 3: DD
            // byte 4(bit 6): 0=lat 1=lon
            // byte 4(bit 7): 0=+(N,E), 1=-(S,W)
            float value = *sensor->value;
            if (value < 0) {
                format |= 1 << 6;
                value *= -1;
            }
            *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
            uint8_t degrees = value;
            uint16_t minutes = (value - degrees) * 60 * 1000;
            *(buffer + *buffer_index + 1) = minutes;
            *(buffer + *buffer_index + 2) = minutes >> 8;
            *(buffer + *buffer_index + 3) = degrees;
            *(buffer + *buffer_index + 4) = format;
            *buffer_index += 5;
        }
    }
}

static void add_sensor_text(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor) {
    uint8_t lenText, lenUnit;
    if (sensor) {
        lenText = strlen(sensor->text);
        lenUnit = strlen(sensor->unit);
    } else {
        lenText = strlen("MSRC");
        lenUnit = strlen("");
    }
    *(buffer + *buffer_index) = sensor_index;
    *(buffer + *buffer_index + 1) = lenText << 3 | lenUnit;
    *buffer_index += 2;
    strcpy((char *)buffer + *buffer_index, sensor ? sensor->text : "MSRC");
    *buffer_index += lenText;
    strcpy((char *)buffer + *buffer_index, sensor ? sensor->unit : "");
    *buffer_index += lenUnit;
}

static int64_t timeout_callback(alarm_id_t id, void *parameters) {
    float *baudrate = (float *)parameters;
    if (*baudrate == 125000L)
        *baudrate = 250000L;
    else
        *baudrate = 125000L;
    uart0_begin(*baudrate, UART_RECEIVER_TX, UART_RECEIVER_RX, JETIEX_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    debug("\nJeti Ex timeout. Baudrate");
    return 0;
}

static uint8_t crc8(uint8_t *crc, uint8_t crc_length) {
    uint8_t crc_up = 0;
    uint8_t c;
    for (c = 0; c < crc_length; c++) {
        crc_up = update_crc8(crc[c], crc_up);
    }
    return crc_up;
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
    uint8_t crc_u;
    uint8_t i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++) {
        crc_u = (crc_u & 0x80) ? 0x07 ^ (crc_u << 1) : (crc_u << 1);
    }
    return crc_u;
}

static uint16_t crc16(uint8_t *p, uint16_t len) {
    uint16_t crc16_data = 0;
    while (len--) {
        crc16_data = update_crc16(crc16_data, p[0]);
        p++;
    }
    return (crc16_data);
}

static uint16_t update_crc16(uint16_t crc, uint8_t data) {
    uint16_t ret_val;
    data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
    data ^= data << 4;
    ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
    return ret_val;
}
