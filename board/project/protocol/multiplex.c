#include "multiplex.h"

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
#include "gps.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "smart_esc.h"
#include "esc_omp_m4.h"
#include "esc_ztw.h"

/* Flysky MULTIPLEX FHSS Data Id */
#define MULTIPLEX_VOLTAGE 1
#define MULTIPLEX_CURRENT 2
#define MULTIPLEX_VARIO 3
#define MULTIPLEX_SPEED 4
#define MULTIPLEX_RPM 5
#define MULTIPLEX_TEMP 6
#define MULTIPLEX_COURSE 7
#define MULTIPLEX_ALTITUDE 8
#define MULTIPLEX_LEVEL 9
#define MULTIPLEX_RSSI 10
#define MULTIPLEX_CONSUMPTION 11
#define MULTIPLEX_FLUID 12
#define MULTIPLEX_DISTANCE 13

#define MULTIPLEX_RECEIVED_NONE 0
#define MULTIPLEX_RECEIVED_POLL 1

#define MULTIPLEX_COMMAND_DISCOVER 0x8
#define MULTIPLEX_COMMAND_TYPE 0x9
#define MULTIPLEX_COMMAND_MEASURE 0xA

#define MULTIPLEX_TIMEOUT_US 1000
#define MULTIPLEX_PACKET_LENGHT 1

typedef struct sensor_multiplex_t {
    uint8_t data_id;
    float *value;
} sensor_multiplex_t;

static void process(sensor_multiplex_t **sensor);
static void send_packet(uint8_t address, sensor_multiplex_t *sensor);
static int16_t format(uint8_t data_id, float value);
static void add_sensor(sensor_multiplex_t *new_sensor, sensor_multiplex_t **sensors);
static void set_config(sensor_multiplex_t **sensors);

void multiplex_task(void *parameters) {
    sensor_multiplex_t *sensor[16] = {NULL};
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(38400, UART_RECEIVER_TX, UART_RECEIVER_RX, MULTIPLEX_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    set_config(sensor);
    debug("\nMultiplex init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(sensor);
    }
}

static void process(sensor_multiplex_t **sensor) {
    uint8_t address = 0;
    if (uart0_available() == MULTIPLEX_PACKET_LENGHT) {
        uart0_read_bytes(&address, MULTIPLEX_PACKET_LENGHT);
        debug("\nMultiplex (%u) < %X", uxTaskGetStackHighWaterMark(NULL), address);

        if (address < 16) {
            send_packet(address, sensor[address]);
        }
    }
}

static void send_packet(uint8_t address, sensor_multiplex_t *sensor) {
    if (!sensor) return;
    uint8_t sensor_id = address << 4 | sensor->data_id;
    uart0_write(sensor_id);
    int16_t value = format(sensor->data_id, *sensor->value);
    uart0_write_bytes((uint8_t *)&value, 2);

    vTaskResume(context.led_task_handle);

    debug("\nMultiplex (%u) > %X %X", uxTaskGetStackHighWaterMark(NULL), sensor_id, value);
}

static void add_sensor(sensor_multiplex_t *new_sensor, sensor_multiplex_t **sensors) {
    static uint8_t sensor_count = 0;
    if (sensor_count < 16) {
        sensors[sensor_count] = new_sensor;
        sensor_count++;
    }
}

static int16_t format(uint8_t data_id, float value) {
    int16_t formatted;
    if (data_id == MULTIPLEX_VOLTAGE || data_id == MULTIPLEX_CURRENT || data_id == MULTIPLEX_VARIO ||
        data_id == MULTIPLEX_SPEED || data_id == MULTIPLEX_TEMP || data_id == MULTIPLEX_COURSE ||
        data_id == MULTIPLEX_DISTANCE)
        formatted = round(value * 10);
    else if (data_id == MULTIPLEX_RPM)
        formatted = round(value / 10);
    else
        formatted = round(value);
    if (formatted > 16383) formatted = 16383;
    if (formatted < -16383) formatted = -16383;
    bool isNegative = false;
    if (formatted < 0) isNegative = true;
    formatted <<= 1;
    if (isNegative) formatted |= 1 << 15;
    return formatted;
}

static void set_config(sensor_multiplex_t **sensors) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_multiplex_t *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_fet};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_fet};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_motor};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_fet};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature_fet};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temp_esc};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temp_esc};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_ALTITUDE, parameter.alt};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_SPEED, parameter.spd};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_DISTANCE, parameter.dist};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.ntc};
        add_sensor(new_sensor, sensors);
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

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_ALTITUDE, parameter.altitude};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
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

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_ALTITUDE, parameter.altitude};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
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

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_ALTITUDE, parameter.altitude};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
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
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){MULTIPLEX_SPEED, parameter.airspeed};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
