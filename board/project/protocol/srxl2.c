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
#include "esc_hw5.h"
#include "esc_kontronik.h"
#include "esc_pwm.h"
#include "fuel_meter.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "srxl.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "xgzp68xxd.h"

#define SRXL2_DEVICE_ID 0x31
#define SRXL2_DEVICE_PRIORITY 10
#define SRXL2_DEVICE_BAUDRATE 0
#define SRXL2_DEVICE_INFO 0
#define SRXL2_DEVICE_UID 0x12345678

static volatile uint8_t dest_id = 0xFF;
static alarm_id_t alarm_id;
static volatile bool send_handshake = false;

static void process(void);
static void send_packet(void);
static void set_config(void);
static int64_t alarm_50ms(alarm_id_t id, void *user_data);

void srxl2_task(void *parameters) {
    sensor = malloc(sizeof(xbus_sensor_t));
    *sensor = (xbus_sensor_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};
    sensor_formatted = malloc(sizeof(xbus_sensor_formatted_t));
    *sensor_formatted = (xbus_sensor_formatted_t){NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
    alarm_id = add_alarm_in_us(50000, alarm_50ms, NULL, true);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    uart0_begin(115200, UART_RECEIVER_TX, UART_RECEIVER_RX, SRXL2_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    set_config();
    debug("\nSRXL2 init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
        if (send_handshake) {
            send_handshake = false;
            srxl2_send_handshake(uart0, SRXL2_DEVICE_ID, 0, SRXL2_DEVICE_PRIORITY, SRXL2_DEVICE_BAUDRATE,
                                 SRXL2_DEVICE_INFO, SRXL2_DEVICE_UID);
        }
    }
}

void srxl2_send_handshake(uart_inst_t *uart, uint8_t source_id, uint8_t dest_id, uint8_t priority, uint8_t baudrate,
                          uint8_t info, uint uid) {
    srxl2_handshake_t handshake;
    handshake.header = SRXL2_HEADER;
    handshake.type = SRXL2_PACKET_TYPE_HANDSHAKE;
    handshake.len = SRXL2_HANDSHAKE_LEN;
    handshake.source_id = source_id;
    handshake.dest_id = dest_id;
    handshake.priority = priority;
    handshake.baudrate = baudrate;
    handshake.info = info;
    handshake.uid = uid;
    uint16_t crc = srxl_get_crc((uint8_t *)&handshake, SRXL2_HANDSHAKE_LEN - 2);
    handshake.crc = swap_16(crc);
    if (uart_get_index(uart))
        uart1_write_bytes((uint8_t *)&handshake, SRXL2_HANDSHAKE_LEN);
    else
        uart0_write_bytes((uint8_t *)&handshake, SRXL2_HANDSHAKE_LEN);
    debug("\nSRXL2 (%u). Send Handshake >", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer((uint8_t *)&handshake, SRXL2_HANDSHAKE_LEN, " 0x%X");
}

static void process(void) {
    uint8_t length = uart0_available();
    if (length) {
        cancel_alarm(alarm_id);
        alarm_id = add_alarm_in_us(50000, alarm_50ms, NULL, true);
        if (length < 3 || length > 128) return;
        uint8_t data[length];
        uint16_t crc;
        uart0_read_bytes(data, length);
        debug("\nSRXL2 (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(data, length, " 0x%X");
        crc = srxl_get_crc(data, length - 2);
        if ((crc >> 8) == data[length - 2] && (crc & 0xFF) == data[length - 1]) {
            debug(" -> CRC OK");
        } else {
            debug(" -> BAD CRC");
            debug(" %X", crc);
            if (dest_id != 0xFF) return;  // allow packets with wrong crc for handshake
        }
        if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[4] == SRXL2_DEVICE_ID) {
            dest_id = data[3];
            debug("\nSRXL2. Set dest_id 0x%X", dest_id);
            srxl2_send_handshake(uart0, SRXL2_DEVICE_ID, dest_id, SRXL2_DEVICE_PRIORITY, SRXL2_DEVICE_BAUDRATE,
                                 SRXL2_DEVICE_INFO, SRXL2_DEVICE_UID);
        } else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_CONTROL && data[4] == SRXL2_DEVICE_ID) {
            send_packet();
        }
    }
}

static void send_packet(void) {
    static uint cont = 0;
    if (!srxl_sensors_count()) return;
    while (!sensor->is_enabled[cont]) {
        cont++;
        if (cont > XBUS_STRU_TELE_DIGITAL_AIR) cont = 0;
    }
    srxl2_telemetry_t packet = {0};
    packet.header = SRXL2_HEADER;
    packet.type = SRXL2_PACKET_TYPE_TELEMETRY;
    packet.len = SRXL2_TELEMETRY_LEN;
    packet.dest_id = dest_id;
    switch (cont) {
        case XBUS_AIRSPEED:
            xbus_format_sensor(XBUS_AIRSPEED_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->airspeed, sizeof(xbus_airspeed_t));
            break;
        case XBUS_BATTERY:
            xbus_format_sensor(XBUS_BATTERY_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->battery, sizeof(xbus_battery_t));
            break;
        case XBUS_ESC:
            xbus_format_sensor(XBUS_ESC_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->esc, sizeof(xbus_esc_t));
            break;
        case XBUS_GPS_LOC:
            xbus_format_sensor(XBUS_GPS_LOC_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->gps_loc, sizeof(xbus_gps_loc_t));
            break;
        case XBUS_GPS_STAT:
            xbus_format_sensor(XBUS_GPS_STAT_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->gps_stat, sizeof(xbus_gps_stat_t));
            break;
        case XBUS_RPMVOLTTEMP:
            xbus_format_sensor(XBUS_RPMVOLTTEMP_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->rpm_volt_temp,
                   sizeof(xbus_rpm_volt_temp_t));
            break;
        case XBUS_FUEL_FLOW:
            xbus_format_sensor(XBUS_FUEL_FLOW_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->fuel_flow, sizeof(xbus_fuel_flow_t));
            break;
        case XBUS_STRU_TELE_DIGITAL_AIR:
            xbus_format_sensor(XBUS_STRU_TELE_DIGITAL_AIR_ID);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)sensor_formatted->stru_tele_digital_air,
                   sizeof(xbus_stru_tele_digital_air_t));
            break;
    }
    uint16_t crc = srxl_get_crc((uint8_t *)&packet, SRXL2_TELEMETRY_LEN - 2);
    packet.crc = swap_16(crc);
    uart0_write_bytes((uint8_t *)&packet, SRXL2_TELEMETRY_LEN);
    cont++;
    debug("\nSRXL2 (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer((uint8_t *)&packet, SRXL2_TELEMETRY_LEN, "0x%X ");
    vTaskResume(context.led_task_handle);
}

static int64_t alarm_50ms(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    send_handshake = true;
    vTaskNotifyGiveIndexedFromISR(context.uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 50000;
}

static void set_config(void) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *baro_temp = NULL, *baro_pressure = NULL;
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
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

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

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor->is_enabled[XBUS_VARIO] = true;
        sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
        *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
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
        sensor_formatted->airspeed = malloc(sizeof(xbus_airspeed_t));
        *sensor_formatted->airspeed = (xbus_airspeed_t){XBUS_AIRSPEED_ID, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->fuel_flow[XBUS_FUEL_FLOW_RATE] = parameter.consumption_instant;
        sensor->fuel_flow[XBUS_FUEL_FLOW_CONSUMED] = parameter.consumption_total;
        sensor->is_enabled[XBUS_FUEL_FLOW] = true;
        sensor_formatted->fuel_flow = calloc(1, 16);
        *sensor_formatted->fuel_flow = (xbus_fuel_flow_t){XBUS_FUEL_FLOW_ID, 0, 0, 0, 0, 0, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_pressure) {
        xgzp68xxd_parameters_t parameter = {config->xgzp68xxd_k, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(xgzp68xxd_task, "fuel_pressure_task", STACK_FUEL_PRESSURE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->stru_tele_digital_air[XBUS_FUEL_PRESSURE] = parameter.pressure;
        sensor->is_enabled[XBUS_STRU_TELE_DIGITAL_AIR] = true;
        sensor_formatted->stru_tele_digital_air = calloc(1, 16);
        *sensor_formatted->stru_tele_digital_air =
            (xbus_stru_tele_digital_air_t){XBUS_STRU_TELE_DIGITAL_AIR_ID, 0, 0, 0};
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
