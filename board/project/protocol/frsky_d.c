#include "frsky_d.h"

#include <math.h>
#include <semphr.h>
#include <stdio.h>
#include <stdlib.h>

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
#include "pwm_out.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "smart_esc.h"
#include "esc_omp_m4.h"
#include "esc_ztw.h"

/* FrSky D Data Id */
#define FRSKY_D_GPS_ALT_BP_ID 0x01
#define FRSKY_D_TEMP1_ID 0x02
#define FRSKY_D_RPM_ID 0x03
#define FRSKY_D_FUEL_ID 0x04
#define FRSKY_D_TEMP2_ID 0x05
#define FRSKY_D_CELL_VOLT_ID 0x06
#define FRSKY_D_GPS_ALT_AP_ID 0x09
#define FRSKY_D_BARO_ALT_BP_ID 0x10
#define FRSKY_D_GPS_SPEED_BP_ID 0x11
#define FRSKY_D_GPS_LONG_BP_ID 0x12
#define FRSKY_D_GPS_LAT_BP_ID 0x13
#define FRSKY_D_GPS_COURS_BP_ID 0x14
#define FRSKY_D_GPS_DAY_MONTH_ID 0x15
#define FRSKY_D_GPS_YEAR_ID 0x16
#define FRSKY_D_GPS_HOUR_MIN_ID 0x17
#define FRSKY_D_GPS_SEC_ID 0x18
#define FRSKY_D_GPS_SPEED_AP_ID 0x19
#define FRSKY_D_GPS_LONG_AP_ID 0x1A
#define FRSKY_D_GPS_LAT_AP_ID 0x1B
#define FRSKY_D_GPS_COURS_AP_ID 0x1C
#define FRSKY_D_BARO_ALT_AP_ID 0x21
#define FRSKY_D_GPS_LONG_EW_ID 0x22
#define FRSKY_D_GPS_LAT_NS_ID 0x23
#define FRSKY_D_ACCEL_X_ID 0x24
#define FRSKY_D_ACCEL_Y_ID 0x25
#define FRSKY_D_ACCEL_Z_ID 0x26
#define FRSKY_D_CURRENT_ID 0x28
#define FRSKY_D_VARIO_ID 0x30
#define FRSKY_D_VFAS_ID 0x39
#define FRSKY_D_VOLTS_BP_ID 0x3A
#define FRSKY_D_VOLTS_AP_ID 0x3B
#define FRSKY_D_FRSKY_LAST_ID 0x3F
#define FRSKY_D_D_RSSI_ID 0xF0
#define FRSKY_D_D_A1_ID 0xF1
#define FRSKY_D_D_A2_ID 0xF2

#define FRSKY_D_INTERVAL 10

typedef struct frsky_d_sensor_parameters_t {
    uint8_t data_id;
    float *value;
    uint16_t rate;

} frsky_d_sensor_parameters_t;

typedef struct frsky_d_sensor_cell_parameters_t {
    float *voltage;
    uint8_t *count;
    uint16_t rate;

} frsky_d_sensor_cell_parameters_t;

static SemaphoreHandle_t semaphore = NULL;

static void sensor_task(void *parameters);
static void send_packet(uint8_t dataId, uint16_t value);
static void send_byte(uint8_t c, bool header);
static uint16_t format(uint8_t data_id, float value);
static void set_config();

void frsky_d_task(void *parameters) {
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(9600, UART_RECEIVER_TX, UART_RECEIVER_RX, 0, 8, 1, UART_PARITY_NONE, true, false);
    semaphore = xSemaphoreCreateMutex();
    set_config();
    debug("\nFrsky D init");
    vTaskSuspend(NULL);
    vTaskDelete(NULL);
}

static void sensor_task(void *parameters) {
    frsky_d_sensor_parameters_t parameter = *(frsky_d_sensor_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore, portMAX_DELAY);
        uint16_t data_formatted = format(parameter.data_id, *parameter.value);
        debug("\nFrSky D. Sensor (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(parameter.data_id, data_formatted);
        xSemaphoreGive(semaphore);
    }
}

static void sensor_cell_task(void *parameters) {
    frsky_d_sensor_cell_parameters_t parameter = *(frsky_d_sensor_cell_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    uint8_t cell_index = 0;
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore, portMAX_DELAY);
        uint value = *parameter.voltage * 50;
        uint16_t data_formatted = (cell_index << 4) | ((value & 0xF00) >> 8) | ((value & 0x0FF) << 8);
        cell_index++;
        cell_index %= *parameter.count;
        debug("\nFrSky D. Sensor cell (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(FRSKY_D_CELL_VOLT_ID, data_formatted);
        xSemaphoreGive(semaphore);
    }
}

static uint16_t format(uint8_t data_id, float value) {
    if (data_id == FRSKY_D_GPS_ALT_BP_ID || data_id == FRSKY_D_BARO_ALT_BP_ID || data_id == FRSKY_D_GPS_SPEED_BP_ID ||
        data_id == FRSKY_D_GPS_COURS_BP_ID)
        return (int16_t)value;

    if (data_id == FRSKY_D_GPS_ALT_AP_ID || data_id == FRSKY_D_BARO_ALT_AP_ID || data_id == FRSKY_D_GPS_SPEED_AP_ID ||
        data_id == FRSKY_D_GPS_COURS_AP_ID)
        return (abs(value) - (int16_t)abs(value)) * 10000;

    if (data_id == FRSKY_D_GPS_LONG_AP_ID || data_id == FRSKY_D_GPS_LAT_AP_ID) {
        float min = fabs(value) * 60;
        return (min - (uint)min) * 10000;
    }

    if (data_id == FRSKY_D_VOLTS_BP_ID) return value * 2;

    if (data_id == FRSKY_D_VOLTS_AP_ID) return ((value * 2) - (int16_t)(value * 2)) * 10000;

    if (data_id == FRSKY_D_GPS_LONG_BP_ID || data_id == FRSKY_D_GPS_LAT_BP_ID) {
        float coord = fabs(value);
        uint8_t deg = coord;
        uint8_t min = (coord - deg) * 60;
        char buf[7];
        sprintf(buf, "%d%d", deg, min);
        return atoi(buf);
    }

    if (data_id == FRSKY_D_GPS_LONG_EW_ID) {
        if (value >= 0) return 'E';
        return 'O';
    }

    if (data_id == FRSKY_D_GPS_LAT_NS_ID) {
        if (value >= 0) return 'N';
        return 'S';
    }

    if (data_id == FRSKY_D_GPS_YEAR_ID) {
        return value / 10000;
    }

    if (data_id == FRSKY_D_GPS_DAY_MONTH_ID) {
        return value - (uint32_t)(value / 10000) * 10000;
    }

    if (data_id == FRSKY_D_GPS_HOUR_MIN_ID) {
        return value / 100;
    }

    if (data_id == FRSKY_D_GPS_SEC_ID) {
        return value - (uint32_t)(value / 100) * 100;
    }

    if (data_id == FRSKY_D_CURRENT_ID || data_id == FRSKY_D_VFAS_ID) return round(value * 10);

    if (data_id == FRSKY_D_RPM_ID) return value / 60;

    return round(value);
}

static void send_byte(uint8_t c, bool header) {
    if ((c == 0x5D || c == 0x5E) && !header) {
        uart0_write(0x5D);
        c ^= 0x60;
    }
    uart0_write(c);
    debug("%X ", c);
}

static void send_packet(uint8_t data_id, uint16_t value) {
    uint8_t *u8p;
    // header
    send_byte(0x5E, true);
    // data_id
    send_byte(data_id, false);
    // value
    u8p = (uint8_t *)&value;
    send_byte(u8p[0], false);
    send_byte(u8p[1], false);
    // footer
    send_byte(0x5E, true);

    // blink
    vTaskResume(context.led_task_handle);
}

static void set_config() {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    frsky_d_sensor_parameters_t parameter_sensor;
    frsky_d_sensor_cell_parameters_t parameter_sensor_cell;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);

        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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
        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP2_ID;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP2_ID;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP2_ID;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /*parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);*/
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temp_esc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP2_ID;
        parameter_sensor.value = parameter.temp_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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
        parameter.bec_voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_omp_m4_task, "esc_omp_m4_task", STACK_ESC_ZTW, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_RPM_ID;
        parameter_sensor.value = parameter.rpm;
        parameter_sensor.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.temp_esc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_TEMP2_ID;
        parameter_sensor.value = parameter.temp_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.voltage = parameter.cell_voltage;
        parameter_sensor_cell.count = parameter.cell_count;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_FRSKY_D_CELL, (void *)&parameter_sensor_cell, 2,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_FUEL_ID;
        parameter_sensor.value = parameter.consumption;
        parameter_sensor.rate = config->refresh_rate_consumption;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_GPS_LONG_BP_ID;
        parameter_sensor.value = parameter.lon;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_LONG_AP_ID;
        parameter_sensor.value = parameter.lon;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_LONG_EW_ID;
        parameter_sensor.value = parameter.lon;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_LAT_BP_ID;
        parameter_sensor.value = parameter.lat;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_LAT_AP_ID;
        parameter_sensor.value = parameter.lat;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_LAT_NS_ID;
        parameter_sensor.value = parameter.lat;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_ALT_BP_ID;
        parameter_sensor.value = parameter.alt;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_ALT_AP_ID;
        parameter_sensor.value = parameter.alt;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_SPEED_BP_ID;
        parameter_sensor.value = parameter.spd;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_SPEED_AP_ID;
        parameter_sensor.value = parameter.spd;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_COURS_BP_ID;
        parameter_sensor.value = parameter.cog;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_COURS_AP_ID;
        parameter_sensor.value = parameter.cog;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_YEAR_ID;
        parameter_sensor.value = parameter.date;
        parameter_sensor.rate = 1000;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_DAY_MONTH_ID;
        parameter_sensor.value = parameter.date;
        parameter_sensor.rate = 1000;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_HOUR_MIN_ID;
        parameter_sensor.value = parameter.time;
        parameter_sensor.rate = 1000;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_SEC_ID;
        parameter_sensor.value = parameter.time;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VARIO_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_BP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VOLTS_AP_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_CURRENT_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /**new_sensor = (sensor_frsky_d_t){FRSKY_D_FUEL_ID, parameter.consumption, config->refresh_rate_consumption};
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);*/
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_TEMP1_ID;
        parameter_sensor.value = parameter.ntc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        parameter_sensor.data_id = FRSKY_D_BARO_ALT_BP_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_BARO_ALT_AP_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VARIO_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = FRSKY_D_BARO_ALT_BP_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_BARO_ALT_AP_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        parameter_sensor.data_id = FRSKY_D_VARIO_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        parameter_sensor.data_id = FRSKY_D_BARO_ALT_BP_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_BARO_ALT_AP_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_VARIO_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
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

        parameter_sensor.data_id = FRSKY_D_GPS_SPEED_BP_ID;
        parameter_sensor.value = parameter.airspeed;
        parameter_sensor.rate = config->refresh_rate_airspeed;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = FRSKY_D_GPS_SPEED_AP_ID;
        parameter_sensor.value = parameter.airspeed;
        parameter_sensor.rate = config->refresh_rate_airspeed;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
