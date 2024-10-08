#include "srxl2.h"

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
#include "esc_kontronik.h"
#include "esc_pwm.h"
#include "esc_vbar.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define SRXL2_TIMEOUT_US 500
#define SRXL2_HEADER 0xA6
#define SRXL2_HANDSHAKE_LEN 14
#define SRXL2_CONTROL_LEN 8
#define SRXL2_TELEMETRY_LEN 22
#define SRXL2_PACKET_TYPE_HANDSHAKE 0x21
#define SRXL2_PACKET_TYPE_CONTROL 0xCD
#define SRXL2_PACKET_TYPE_TELEMETRY 0x80
#define SRXL2_DESTID 0x10

static uint8_t dest_id;

static void process();
static void send_packet();
static uint16_t get_crc(uint8_t *buffer, uint8_t lenght);
static uint16_t byte_crc(uint16_t crc, uint8_t new_byte);
static void set_config();
static void send_handshake();

void srxl2_task(void *parameters) {
    sensor = malloc(sizeof(xbus_sensor_t));
    *sensor = (xbus_sensor_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};
    sensor_formatted = malloc(sizeof(xbus_sensor_formatted_t));
    *sensor_formatted = (xbus_sensor_formatted_t){NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    uart0_begin(115200, UART_RECEIVER_TX, UART_RECEIVER_RX, SRXL2_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    set_config();
    debug("\nSRXL2 init");
    while (1) {
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static void process() {
    static bool mute = true;
    uint8_t length = uart0_available();
    if (length) {
        uint8_t data[length];
        uart0_read_bytes(data, length);
        debug("\nSRXL2 (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(data, length, " 0x%X");
        if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[4] == SRXL2_DESTID) {
            send_handshake(data);
        } else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_CONTROL && data[4] == SRXL2_DESTID) {
            if (!mute) send_packet();
            mute = !mute;
        }
    }
}

static void send_handshake(uint8_t *handshake_packet) {
    uint8_t buffer[SRXL2_HANDSHAKE_LEN];
    buffer[0] = 0xA6;                 // header
    buffer[1] = 0x21;                 // packet type
    buffer[2] = 14;                   // length
    buffer[3] = handshake_packet[3];  // source Id
    buffer[4] = handshake_packet[4];  // dest Id
    dest_id = handshake_packet[4];
    ;
    buffer[5] = 10;                   // priority (0-100)
    buffer[6] = 0;                    // baudrate: 0 = 115200, 1 = 400000
    buffer[7] = handshake_packet[4];  // info
    buffer[8] = 0;                    // uid
    buffer[9] = 0;                    // uid
    buffer[10] = 0;                   // uid
    buffer[11] = 1;                   // uid
    uint16_t crc;
    crc = get_crc(buffer, SRXL2_HANDSHAKE_LEN - 2);  // all bytes, including header
    buffer[12] = crc >> 8;
    buffer[13] = crc;
    uart0_write_bytes((uint8_t *)&crc, 2);
    debug("\nSRXL2. Send Handshake >");
    debug_buffer(buffer, SRXL2_HANDSHAKE_LEN, " 0x%X");
    vTaskResume(context.led_task_handle);
}

static void send_packet() {
    static uint cont = 0;
    uint max_cont = 0;
    while (sensor->is_enabled[cont] == false && max_cont < XBUS_RPMVOLTTEMP) {
        cont++;
        max_cont++;
        if (cont > XBUS_RPMVOLTTEMP) cont = 0;
    }
    if (max_cont == XBUS_RPMVOLTTEMP) return;
    uint8_t buffer[SRXL2_TELEMETRY_LEN] = {0};
    buffer[0] = SRXL2_HEADER;
    buffer[1] = SRXL2_PACKET_TYPE_TELEMETRY;
    buffer[2] = SRXL2_TELEMETRY_LEN;
    buffer[3] = dest_id;
    switch (cont) {
        case XBUS_AIRSPEED:
            xbus_format_sensor(XBUS_AIRSPEED_ID);
            memcpy((uint8_t *)&buffer[4], (uint8_t *)&sensor_formatted->airspeed, sizeof(xbus_airspeed_t));
            break;
        case XBUS_BATTERY:
            xbus_format_sensor(XBUS_AIRSPEED_ID);
            memcpy((uint8_t *)&buffer[4], (uint8_t *)&sensor_formatted->battery, sizeof(xbus_battery_t));
            break;
        case XBUS_ESC:
            xbus_format_sensor(XBUS_ESC_ID);
            memcpy((uint8_t *)&buffer[4], (uint8_t *)&sensor_formatted->esc, sizeof(xbus_esc_t));
            break;
        case XBUS_GPS_LOC:
            xbus_format_sensor(XBUS_GPS_LOC_ID);
            memcpy((uint8_t *)&buffer[4], (uint8_t *)&sensor_formatted->gps_loc, sizeof(xbus_gps_loc_t));
            break;
        case XBUS_GPS_STAT:
            xbus_format_sensor(XBUS_GPS_STAT_ID);
            memcpy((uint8_t *)&buffer[4], (uint8_t *)&sensor_formatted->gps_stat, sizeof(xbus_gps_stat_t));
            break;
        case XBUS_RPMVOLTTEMP:
            xbus_format_sensor(XBUS_RPMVOLTTEMP_ID);
            memcpy((uint8_t *)&buffer[4], (uint8_t *)&sensor_formatted->rpm_volt_temp, sizeof(xbus_rpm_volt_temp_t));
            break;
    }
    uint16_t crc;
    crc = (get_crc(buffer, SRXL2_TELEMETRY_LEN - 2));  // all bytes, including header
    buffer[20] = crc >> 8;
    buffer[21] = crc;
    uart0_write_bytes(buffer, SRXL2_TELEMETRY_LEN);
    cont++;
    debug("\nSRXL2 (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(buffer, SRXL2_TELEMETRY_LEN, "0x%X ");
    vTaskResume(context.led_task_handle);
}

static uint16_t get_crc(uint8_t *buffer, uint8_t lenght) {
    uint16_t crc = 0;
    for (int i = 0; i < lenght; i++) crc += byte_crc(crc, buffer[i]);
    return crc;
}

static uint16_t byte_crc(uint16_t crc, uint8_t new_byte) {
    uint8_t loop;
    crc = crc ^ (uint16_t)new_byte << 8;
    for (loop = 0; loop < 8; loop++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void set_config() {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW4) {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier,
                                          config->enable_pwm_out,
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
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    if (config->esc_protocol == ESC_VBAR) {
        esc_vbar_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_vbar_task, "esc_vbar_task", STACK_ESC_VBAR, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor->esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
        sensor_formatted->gps_loc = malloc(sizeof(xbus_gps_loc_t));
        *sensor_formatted->gps_loc = (xbus_gps_loc_t){XBUS_GPS_LOC_ID, 0, 0, 0, 0, 0, 0, 0};
        sensor_formatted->gps_stat = malloc(sizeof(xbus_gps_stat_t));
        *sensor_formatted->gps_stat = (xbus_gps_stat_t){XBUS_GPS_STAT_ID, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] = parameter.voltage;
        sensor->is_enabled[XBUS_RPMVOLTTEMP] = true;
        sensor_formatted->rpm_volt_temp = malloc(sizeof(xbus_rpm_volt_temp_t));
        *sensor_formatted->rpm_volt_temp = (xbus_rpm_volt_temp_t){XBUS_RPMVOLTTEMP_ID, 0, 0, 0, 0};
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
        sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        sensor->is_enabled[XBUS_BATTERY] = true;
        sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        *sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP] = parameter.ntc;
        sensor->is_enabled[XBUS_RPMVOLTTEMP] = true;
        sensor_formatted->rpm_volt_temp = malloc(sizeof(xbus_rpm_volt_temp_t));
        *sensor_formatted->rpm_volt_temp = (xbus_rpm_volt_temp_t){XBUS_RPMVOLTTEMP_ID, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3, config->analog_rate, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->airspeed[XBUS_AIRSPEED_AIRSPEED] = parameter.airspeed;
        sensor->is_enabled[XBUS_AIRSPEED] = true;
        sensor_formatted->airspeed = malloc(sizeof(xbus_airspeed_t));
        *sensor_formatted->airspeed = (xbus_airspeed_t){XBUS_AIRSPEED_ID, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
