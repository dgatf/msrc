#include "xbus.h"

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
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)

xbus_sensor_t *sensor;
xbus_sensor_formatted_t *sensor_formatted;

static void i2c_request_handler(uint8_t address);
static void set_config();
static uint8_t bcd8(float value, uint8_t precision);
static uint16_t bcd16(float value, uint8_t precision);
static uint32_t bcd32(float value, uint8_t precision);
static int64_t interval_250_callback(alarm_id_t id, void *parameters);
static int64_t interval_500_callback(alarm_id_t id, void *parameters);
static int64_t interval_1000_callback(alarm_id_t id, void *parameters);
static int64_t interval_1500_callback(alarm_id_t id, void *parameters);
static int64_t interval_2000_callback(alarm_id_t id, void *parameters);
static int64_t interval_3000_callback(alarm_id_t id, void *parameters);

void xbus_i2c_handler(uint8_t address) { i2c_request_handler(address); }

void xbus_task(void *parameters) {
    sensor = malloc(sizeof(xbus_sensor_t));
    *sensor = (xbus_sensor_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};
    sensor_formatted = malloc(sizeof(xbus_sensor_formatted_t));
    *sensor_formatted = (xbus_sensor_formatted_t){NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    PIO pio = pio1;
    uint pin = I2C1_SDA_GPIO;

    i2c_multi_init(pio, pin);
    i2c_multi_set_request_handler(i2c_request_handler);

    set_config();

    debug("\nXBUS init");

    vTaskSuspend(NULL);
    free(sensor);
    vTaskDelete(NULL);
}

void xbus_format_sensor(uint8_t address) {
    static float alt = 0;
    switch (address) {
        case XBUS_AIRSPEED_ID: {
            sensor_formatted->airspeed->airspeed = swap_16((uint16_t)(*sensor->airspeed[XBUS_AIRSPEED_AIRSPEED]));
            if (swap_16((uint16_t)(sensor_formatted->airspeed->airspeed)) >
                swap_16((uint16_t)(sensor_formatted->airspeed->max_airspeed)))
                sensor_formatted->airspeed->max_airspeed = sensor_formatted->airspeed->airspeed;
            break;
        }
        case XBUS_GPS_LOC_ID: {
            uint8_t gps_flags = 0;
            float lat = *sensor->gps_loc[XBUS_GPS_LOC_LATITUDE];
            if (lat < 0)  // N=1,+, S=0,-
                lat *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT;
            uint deg = lat / 60;
            float min = lat - deg * 60;
            sensor_formatted->gps_loc->latitude = ((uint32_t)bcd8(deg, 0) << 24) | bcd32(min, 4);
            float lon = *sensor->gps_loc[XBUS_GPS_LOC_LONGITUDE];
            if (lon < 0)  // E=1,+, W=0,-
                lon *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_EAST_BIT;
            if (lon >= 6000) {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
                lon -= 6000;
            }
            deg = lon / 60;
            min = lon - deg * 60;
            sensor_formatted->gps_loc->longitude = ((uint32_t)bcd8(deg, 0) << 24) | bcd32(min, 4);
            sensor_formatted->gps_loc->course = bcd16(*sensor->gps_loc[XBUS_GPS_LOC_COURSE], 1);
            sensor_formatted->gps_loc->hdop = bcd8(*sensor->gps_loc[XBUS_GPS_LOC_HDOP], 1);
            alt = *sensor->gps_loc[XBUS_GPS_LOC_ALTITUDE];
            if (alt < 0) {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_NEGATIVE_ALT_BIT;
                alt *= -1;
            }
            sensor_formatted->gps_loc->gps_flags = gps_flags;
            sensor_formatted->gps_loc->altitude_low = bcd16(fmod(alt, 1000), 1);
            break;
        }
        case XBUS_GPS_STAT_ID: {
            sensor_formatted->gps_stat->speed = bcd16(*sensor->gps_stat[XBUS_GPS_STAT_SPEED], 1);
            sensor_formatted->gps_stat->utc = bcd32(*sensor->gps_stat[XBUS_GPS_STAT_TIME], 0) << 8;
            sensor_formatted->gps_stat->num_sats = bcd8(*sensor->gps_stat[XBUS_GPS_STAT_SATS], 0);
            sensor_formatted->gps_stat->altitude_high = bcd8((uint8_t)(alt / 1000), 0);
            break;
        }
        case XBUS_ENERGY_ID: {
            if (sensor->energy[XBUS_ENERGY_CURRENT1])
                sensor_formatted->energy->current_a = swap_16((int16_t)(*sensor->energy[XBUS_ENERGY_CURRENT1] * 100));
            if (sensor->energy[XBUS_ENERGY_CONSUMPTION1])
                sensor_formatted->energy->charge_used_a =
                    swap_16((int16_t)(*sensor->energy[XBUS_ENERGY_CONSUMPTION1] * 10));
            if (sensor->energy[XBUS_ENERGY_VOLTAGE1])
                sensor_formatted->energy->volts_a = swap_16((uint16_t)(*sensor->energy[XBUS_ENERGY_VOLTAGE1] * 100));
            if (sensor->energy[XBUS_ENERGY_CURRENT2])
                sensor_formatted->energy->current_b = swap_16((int16_t)(*sensor->energy[XBUS_ENERGY_CURRENT2] * 10));
            if (sensor->energy[XBUS_ENERGY_CONSUMPTION2])
                sensor_formatted->energy->charge_used_b = swap_16((int16_t)(*sensor->energy[XBUS_ENERGY_CONSUMPTION2]));
            if (sensor->energy[XBUS_ENERGY_VOLTAGE2])
                sensor_formatted->energy->volts_b = swap_16((uint16_t)(*sensor->energy[XBUS_ENERGY_VOLTAGE2] * 100));
            break;
        }
        case XBUS_ESC_ID: {
            if (sensor->esc[XBUS_ESC_RPM])
                sensor_formatted->esc->rpm = swap_16((uint16_t)(*sensor->esc[XBUS_ESC_RPM] / 10));
            if (sensor->esc[XBUS_ESC_VOLTAGE])
                sensor_formatted->esc->volts_input = swap_16((uint16_t)(*sensor->esc[XBUS_ESC_VOLTAGE] * 100));
            if (sensor->esc[XBUS_ESC_TEMPERATURE_FET])
                sensor_formatted->esc->temp_fet = swap_16((uint16_t)(*sensor->esc[XBUS_ESC_TEMPERATURE_FET] * 10));
            if (sensor->esc[XBUS_ESC_CURRENT])
                sensor_formatted->esc->current_motor = swap_16((uint16_t)(*sensor->esc[XBUS_ESC_CURRENT] * 100));
            if (sensor->esc[XBUS_ESC_TEMPERATURE_BEC])
                sensor_formatted->esc->temp_bec = swap_16((uint16_t)(*sensor->esc[XBUS_ESC_TEMPERATURE_BEC] * 10));
            if (sensor->esc[XBUS_ESC_CURRENT_BEC])
                sensor_formatted->esc->current_bec = *sensor->esc[XBUS_ESC_CURRENT_BEC] * 10;
            if (sensor->esc[XBUS_ESC_VOLTAGE_BEC])
                sensor_formatted->esc->voltage_bec = *sensor->esc[XBUS_ESC_VOLTAGE_BEC] * 20;
            break;
        }
        case XBUS_BATTERY_ID: {
            if (sensor->battery[XBUS_BATTERY_CURRENT1])
                sensor_formatted->battery->current_a = swap_16((int16_t)(*sensor->battery[XBUS_BATTERY_CURRENT1] * 10));
            if (sensor->battery[XBUS_BATTERY_CONSUMPTION1])
                sensor_formatted->battery->charge_used_a =
                    swap_16((int16_t)(*sensor->battery[XBUS_BATTERY_CONSUMPTION1]));
            if (sensor->battery[XBUS_BATTERY_TEMP1])
                sensor_formatted->battery->temp_a = swap_16((uint16_t)(*sensor->battery[XBUS_BATTERY_TEMP1] * 10));
            if (sensor->battery[XBUS_BATTERY_CURRENT2])
                sensor_formatted->battery->current_b = swap_16((int16_t)(*sensor->battery[XBUS_BATTERY_CURRENT2] * 10));
            if (sensor->battery[XBUS_BATTERY_CONSUMPTION2])
                sensor_formatted->battery->charge_used_b =
                    swap_16((int16_t)(*sensor->battery[XBUS_BATTERY_CONSUMPTION2]));
            if (sensor->battery[XBUS_BATTERY_TEMP2])
                sensor_formatted->battery->temp_b = swap_16((uint16_t)(*sensor->battery[XBUS_BATTERY_TEMP2] * 10));
            break;
        }
        case XBUS_VARIO_ID: {
            float altitude = *sensor->vario[XBUS_VARIO_ALTITUDE];
            sensor_formatted->vario->altitude = swap_16((int16_t)(altitude * 10));
#ifdef SIM_SENSORS
            sensor_formatted->vario->delta_0250ms = swap_16((int16_t)(-10));
            sensor_formatted->vario->delta_0500ms = swap_16((int16_t)(20));
            sensor_formatted->vario->delta_1000ms = swap_16((int16_t)(-12.32));
            sensor_formatted->vario->delta_1500ms = swap_16((int16_t)(15));
            sensor_formatted->vario->delta_2000ms = swap_16((int16_t)(20));
            sensor_formatted->vario->delta_3000ms = swap_16((int16_t)(-300));
#endif
            break;
        }
        case XBUS_RPMVOLTTEMP_ID: {
            if (sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT])
                sensor_formatted->rpm_volt_temp->volts =
                    swap_16((uint16_t)(*sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] * 100));
            if (sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP])
                sensor_formatted->rpm_volt_temp->temperature =
                    swap_16((int16_t)(*sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP]));
            break;
        }
    }
}

static void i2c_request_handler(uint8_t address) {
    uint8_t buffer[64];

    debug("\nXBUS (%u) Address: %X Packet: ", uxTaskGetStackHighWaterMark(context.receiver_task_handle), address);

    switch (address) {
        case XBUS_AIRSPEED_ID:
            if (!sensor->is_enabled[XBUS_AIRSPEED]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->airspeed);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->airspeed, sizeof(xbus_airspeed_t), "0x%X ");
            break;
        case XBUS_ALTIMETER_ID:
            if (!sensor->is_enabled[XBUS_ALTIMETER]) break;
            break;
        case XBUS_GPS_LOC_ID:
            if (!sensor->is_enabled[XBUS_GPS_LOC]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->gps_loc);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->gps_loc, sizeof(xbus_gps_loc_t), "0x%X ");
            break;
        case XBUS_GPS_STAT_ID:
            if (!sensor->is_enabled[XBUS_GPS_STAT]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->gps_stat);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->gps_stat, sizeof(xbus_gps_stat_t), "0x%X ");
            break;
        case XBUS_ENERGY_ID:
            if (!sensor->is_enabled[XBUS_ENERGY]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->energy);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->airspeed, sizeof(xbus_energy_t), "0x%X ");
            break;
        case XBUS_ESC_ID:
            if (!sensor->is_enabled[XBUS_ESC]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->esc);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->esc, sizeof(xbus_esc_t), "0x%X ");
            break;
        case XBUS_BATTERY_ID:
            if (!sensor->is_enabled[XBUS_BATTERY]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->battery);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->battery, sizeof(xbus_battery_t), "0x%X ");
            break;
        case XBUS_VARIO_ID:
            if (!sensor->is_enabled[XBUS_VARIO]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->vario);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->vario, sizeof(xbus_vario_t), "0x%X ");
            break;
        case XBUS_RPMVOLTTEMP_ID:
            if (!sensor->is_enabled[XBUS_RPMVOLTTEMP]) break;
            xbus_format_sensor(address);
            i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->rpm_volt_temp);
            vTaskResume(context.led_task_handle);
            debug_buffer((uint8_t *)sensor_formatted->rpm_volt_temp, sizeof(xbus_rpm_volt_temp_t), "0x%X ");
            break;
    }
}

static void set_config() {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

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
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor->esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);
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
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor->esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);
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
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor->esc[XBUS_ESC_CURRENT_BEC] = parameter.current_bec;
        sensor->esc[XBUS_ESC_VOLTAGE_BEC] = parameter.voltage_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

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

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor->esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor->esc[XBUS_ESC_CURRENT_BEC] = parameter.current_bec;
        sensor->esc[XBUS_ESC_VOLTAGE_BEC] = parameter.voltage_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

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

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

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

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = calloc(1, 16);
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps) {
        nmea_parameters_t parameter = {config->gps_baudrate,  malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;

        sensor->gps_loc[XBUS_GPS_LOC_ALTITUDE] = parameter.alt;
        sensor->gps_loc[XBUS_GPS_LOC_LATITUDE] = parameter.lat;
        sensor->gps_loc[XBUS_GPS_LOC_LONGITUDE] = parameter.lon;
        sensor->gps_loc[XBUS_GPS_LOC_COURSE] = parameter.cog;
        sensor->gps_loc[XBUS_GPS_LOC_HDOP] = parameter.hdop;
        sensor->gps_stat[XBUS_GPS_STAT_SPEED] = parameter.spd;
        sensor->gps_stat[XBUS_GPS_STAT_TIME] = parameter.time;
        sensor->gps_stat[XBUS_GPS_STAT_SATS] = parameter.sat;
        sensor->gps_stat[XBUS_GPS_STAT_ALTITUDE] = parameter.alt;
        sensor->is_enabled[XBUS_GPS_LOC] = true;
        sensor->is_enabled[XBUS_GPS_STAT] = true;
        sensor_formatted->gps_loc = calloc(1, 16);
        *sensor_formatted->gps_loc = (xbus_gps_loc_t){XBUS_GPS_LOC_ID, 0, 0, 0, 0, 0, 0, 0};
        sensor_formatted->gps_stat = calloc(1, 16);
        *sensor_formatted->gps_stat = (xbus_gps_stat_t){XBUS_GPS_STAT_ID, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_GPS_LOC_ID);
        i2c_multi_enable_address(XBUS_GPS_STAT_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        if (!config->xbus_use_alternative_volt_temp) {
            sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] = parameter.voltage;
            sensor->is_enabled[XBUS_RPMVOLTTEMP] = true;
            sensor_formatted->rpm_volt_temp = calloc(1, 16);
            *sensor_formatted->rpm_volt_temp = (xbus_rpm_volt_temp_t){XBUS_RPMVOLTTEMP_ID, 0, 0, 0, 0};
            i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
        } else {
            sensor->energy[XBUS_ENERGY_VOLTAGE1] = parameter.voltage;
            sensor->is_enabled[XBUS_ENERGY] = true;
            sensor_formatted->energy = calloc(1, 16);
            *sensor_formatted->energy = (xbus_energy_t){XBUS_ENERGY_ID, 0, 0, 0, 0, 0, 0};
            i2c_multi_enable_address(XBUS_ENERGY_ID);
        }

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

        sensor->energy[XBUS_ENERGY_CURRENT1] = parameter.current;
        sensor->energy[XBUS_ENERGY_CONSUMPTION1] = parameter.consumption;
        if (!sensor->is_enabled[XBUS_ENERGY]) {
            sensor->is_enabled[XBUS_ENERGY] = true;
            sensor_formatted->energy = calloc(1, 16);
            *sensor_formatted->energy = (xbus_energy_t){XBUS_ENERGY_ID, 0, 0, 0, 0, 0, 0};
        }
        i2c_multi_enable_address(XBUS_ENERGY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        if (!config->xbus_use_alternative_volt_temp) {
            sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP] = parameter.ntc;
            sensor->is_enabled[XBUS_RPMVOLTTEMP] = true;
            sensor_formatted->rpm_volt_temp = calloc(1, 16);
            *sensor_formatted->rpm_volt_temp = (xbus_rpm_volt_temp_t){XBUS_RPMVOLTTEMP_ID, 0, 0, 0, 0};
            i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
        } else {
            sensor->battery[XBUS_BATTERY_TEMP1] = parameter.ntc;
            sensor->is_enabled[XBUS_BATTERY] = true;
            sensor_formatted->battery = calloc(1, 16);
            *sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0};
            i2c_multi_enable_address(XBUS_BATTERY_ID);
        }
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
        
        add_alarm_in_ms(250, interval_250_callback, NULL, false);
        add_alarm_in_ms(500, interval_500_callback, NULL, false);
        add_alarm_in_ms(1000, interval_1000_callback, NULL, false);
        add_alarm_in_ms(1500, interval_1500_callback, NULL, false);
        add_alarm_in_ms(2000, interval_2000_callback, NULL, false);
        add_alarm_in_ms(3000, interval_3000_callback, NULL, false);
        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = calloc(1, 16);
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_VARIO_ID);

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
        
        add_alarm_in_ms(250, interval_250_callback, NULL, false);
        add_alarm_in_ms(500, interval_500_callback, NULL, false);
        add_alarm_in_ms(1000, interval_1000_callback, NULL, false);
        add_alarm_in_ms(1500, interval_1500_callback, NULL, false);
        add_alarm_in_ms(2000, interval_2000_callback, NULL, false);
        add_alarm_in_ms(3000, interval_3000_callback, NULL, false);
        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = calloc(1, 16);
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        add_alarm_in_ms(250, interval_250_callback, NULL, false);
        add_alarm_in_ms(500, interval_500_callback, NULL, false);
        add_alarm_in_ms(1000, interval_1000_callback, NULL, false);
        add_alarm_in_ms(1500, interval_1500_callback, NULL, false);
        add_alarm_in_ms(2000, interval_2000_callback, NULL, false);
        add_alarm_in_ms(3000, interval_3000_callback, NULL, false);
        
        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }
        
        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = calloc(1, 16);
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 100,
                                           (float)config->airspeed_slope / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->airspeed[XBUS_AIRSPEED_AIRSPEED] = parameter.airspeed;
        sensor->is_enabled[XBUS_AIRSPEED] = true;
        sensor_formatted->airspeed = calloc(1, 16);
        *sensor_formatted->airspeed = (xbus_airspeed_t){XBUS_AIRSPEED_ID, 0, 0, 0};
        i2c_multi_enable_address(XBUS_AIRSPEED_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->xbus_clock_stretch) {
        gpio_set_dir(CLOCK_STRETCH_GPIO, true);
        gpio_put(CLOCK_STRETCH_GPIO, false);
    }
}

static uint8_t bcd8(float value, uint8_t precision) {
    char buf[10] = {0};
    uint8_t output = 0;
    for (int i = 0; i < precision; i++) value = value * 10;
    sprintf(buf, "%02i", (uint8_t)value);
    for (int i = 0; i < 2; i++) output |= (buf[i] - 48) << ((1 - i) * 4);
    return output;
}

static uint16_t bcd16(float value, uint8_t precision) {
    char buf[10] = {0};
    uint16_t output = 0;
    for (int i = 0; i < precision; i++) value = value * 10;
    sprintf(buf, "%04i", (uint16_t)value);
    for (int i = 0; i < 4; i++) output |= (uint16_t)(buf[i] - 48) << ((3 - i) * 4);
    return output;
}

static uint32_t bcd32(float value, uint8_t precision) {
    char buf[10] = {0};
    uint32_t output = 0;
    for (int i = 0; i < precision; i++) value = value * 10;
    sprintf(buf, "%08li", (uint32_t)value);
    for (int i = 0; i < 8; i++) output |= (uint32_t)(buf[i] - 48) << ((7 - i) * 4);
    return output;
}

static int64_t interval_250_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensor_formatted->vario->delta_0250ms = swap_16((int16_t)(round(*sensor->vario[XBUS_VARIO_ALTITUDE] - prev) * 10));
    prev = *sensor->vario[XBUS_VARIO_ALTITUDE];
    return 250000L;
}

static int64_t interval_500_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensor_formatted->vario->delta_0500ms = swap_16((int16_t)(round(*sensor->vario[XBUS_VARIO_ALTITUDE] - prev) * 10));
    prev = *sensor->vario[XBUS_VARIO_ALTITUDE];
    return 500000L;
}

static int64_t interval_1000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensor_formatted->vario->delta_1000ms = swap_16((int16_t)round(*sensor->vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor->vario[XBUS_VARIO_ALTITUDE];
    return 1000000L;
}

static int64_t interval_1500_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensor_formatted->vario->delta_1500ms = swap_16((int16_t)round(*sensor->vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor->vario[XBUS_VARIO_ALTITUDE];
    return 1500000L;
}

static int64_t interval_2000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensor_formatted->vario->delta_2000ms = swap_16((int16_t)round(*sensor->vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor->vario[XBUS_VARIO_ALTITUDE];
    return 2000000L;
}

static int64_t interval_3000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensor_formatted->vario->delta_3000ms = swap_16((int16_t)round(*sensor->vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor->vario[XBUS_VARIO_ALTITUDE];
    return 3000000L;
}
