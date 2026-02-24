#include "ghst.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "gps.h"
#include "ibus.h"
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define GHST_DL_PACK_STAT 0x23      // Battery Status
#define GHST_DL_GPS_PRIMARY 0x25    // Primary GPS Data
#define GHST_DL_GPS_SECONDARY 0x26  // Secondary GPS Data
#define GHST_DL_MAGBARO 0x27        // Magnetometer, Barometer (and Vario) Data
#define GHST_DL_MSP_RESP 0x28       // MSP Response

#define GHST_ADDR_RX 0x89

#define GHST_PACKET_LEN 0x0C

#define GHST_TIMEOUT_US 500

#define TYPE_PACK_STAT 0
#define TYPE_GPS_PRIMARY 1
#define TYPE_GPS_SECONDARY 2
#define TYPE_MAGBARO 3

#define MAX_SENSORS 4

typedef struct ghst_sensor_pack_stat_formatted_t {
    uint16_t voltage;   // 10mV
    uint16_t current;   // 10mA
    uint16_t consumed;  // 10mAh
    uint8_t rx_volt;    // 100mV
} __attribute__((packed)) ghst_sensor_pack_stat_formatted_t;

typedef struct ghst_sensor_gps_primary_formatted_t {
    uint32_t latitude;   // degree / 10,000,000
    uint32_t longitude;  // degree / 10,000,000
    int16_t altitude;    // meters
} __attribute__((packed)) ghst_sensor_gps_primary_formatted_t;

typedef struct ghst_sensor_gps_secondary_formatted_t {
    uint16_t groundspeed;  // cm/s
    uint16_t heading;      // GPS heading, degree/10
    uint8_t satellites;    // satellites
    uint16_t distance;     // 10m
    uint16_t homedir;      // degree/10
    uint8_t flags;         // fix 0x01, fix home 0x02
} __attribute__((packed)) ghst_sensor_gps_secondary_formatted_t;

typedef struct ghst_sensor_magbaro_formatted_t {
    uint16_t heading;  // degree/10
    int16_t altitude;  // m
    int16_t vario;     // cm/s
    uint8_t flags;     // maghead 0x01, baroalt 0x02, vario 0x04
} __attribute__((packed)) ghst_sensor_magbaro_formatted_t;

typedef struct ghst_sensor_pack_stat_t {
    float *voltage;
    float *current;
    float *consumed;
    float *rx_volt;
} ghst_sensor_pack_stat_t;

typedef struct ghst_sensor_gps_primary_t {
    float *latitude;
    float *longitude;
    float *altitude;
} ghst_sensor_gps_primary_t;

typedef struct ghst_sensor_gps_secondary_t {
    float *groundspeed;
    float *heading;
    float *satellites;
    float *distance;
    float *homedir;
    float *flags;
} ghst_sensor_gps_secondary_t;

typedef struct ghst_sensor_magbaro_t {
    float *heading;
    float *altitude;
    float *vario;
    float *flags;
} ghst_sensor_magbaro_t;

typedef struct ghst_sensors_t {
    bool enabled_sensors[MAX_SENSORS];
    ghst_sensor_pack_stat_t pack_stat;
    ghst_sensor_gps_primary_t gps_primary;
    ghst_sensor_gps_secondary_t gps_secondary;
    ghst_sensor_magbaro_t magbaro;
} ghst_sensors_t;

static void process(ghst_sensors_t *sensors);
static uint8_t format_sensor(ghst_sensors_t *sensors, uint8_t type, uint8_t *buffer);
static void send_packet(ghst_sensors_t *sensors);
static uint8_t get_crc(const uint8_t *ptr, uint32_t len);
static uint8_t crc8(uint8_t crc, unsigned char a);
static void set_config(ghst_sensors_t *sensors);
static inline uint sensor_count(ghst_sensors_t *sensors);

void ghst_task(void *parameters) {
    ghst_sensors_t sensors = {0};
    set_config(&sensors);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(420000L, UART_RECEIVER_TX, UART_RECEIVER_RX, GHST_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    debug("\nGHST init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&sensors);
    }
}

static void process(ghst_sensors_t *sensors) {
    if (uart0_available() == GHST_PACKET_LEN + 2) {
        uint8_t data[GHST_PACKET_LEN + 2];
        uart0_read_bytes(data, GHST_PACKET_LEN + 2);
        debug("\nGHST (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(data, GHST_PACKET_LEN + 2, "0x%X ");
        uint8_t crc = get_crc(data + 2, GHST_PACKET_LEN - 1); // crc from type, size 11
        // send telemetry
        if (data[0] == GHST_ADDR_RX) {
            if (data[13] == crc)
                send_packet(sensors);
            else
                debug("\nGHST. Bad CRC 0x%X - 0x%X", data[13], crc);
        }
    }
}

static uint8_t format_sensor(ghst_sensors_t *sensors, uint8_t type, uint8_t *buffer) {
    // Packet format: [rx_addr] [len] [type] [payload] [crc8 from type]
    buffer[0] = GHST_ADDR_RX;
    buffer[1] = GHST_PACKET_LEN;
    switch (type) {
        case TYPE_PACK_STAT: {
            buffer[2] = GHST_DL_PACK_STAT;
            ghst_sensor_pack_stat_formatted_t sensor = {0};
            if (sensors->pack_stat.voltage) sensor.voltage = *sensors->pack_stat.voltage * 100;
            if (sensors->pack_stat.current) sensor.current = *sensors->pack_stat.current * 100;
            if (sensors->pack_stat.consumed) sensor.consumed = *sensors->pack_stat.consumed / 10;
            memcpy(&buffer[3], &sensor, sizeof(ghst_sensor_pack_stat_formatted_t));
            buffer[GHST_PACKET_LEN + 1] = get_crc(&buffer[2], GHST_PACKET_LEN + 1);
            break;
        }
        case TYPE_GPS_PRIMARY: {
            buffer[2] = GHST_DL_GPS_PRIMARY;
            ghst_sensor_gps_primary_formatted_t sensor = {0};
            if (sensors->gps_primary.latitude) sensor.latitude = *sensors->gps_primary.latitude * 10000000L;
            if (sensors->gps_primary.longitude) sensor.longitude = *sensors->gps_primary.longitude * 10000000L;
            if (sensors->gps_primary.altitude) sensor.altitude = *sensors->gps_primary.altitude;
            memcpy(&buffer[3], &sensor, sizeof(ghst_sensor_gps_primary_formatted_t));
            buffer[GHST_PACKET_LEN + 1] = get_crc(&buffer[2], GHST_PACKET_LEN + 1);
            break;
        }
        case TYPE_GPS_SECONDARY: {
            buffer[2] = GHST_DL_GPS_SECONDARY;
            ghst_sensor_gps_secondary_formatted_t sensor = {0};
            if (sensors->gps_secondary.groundspeed)
                sensor.groundspeed = fabs(*sensors->gps_secondary.groundspeed * 1000.0F / 36);
            if (sensors->gps_secondary.heading) sensor.heading = *sensors->gps_secondary.heading * 10;
            if (sensors->gps_secondary.satellites) sensor.satellites = *sensors->gps_secondary.satellites;
            if (sensors->gps_secondary.distance) sensor.distance = *sensors->gps_secondary.distance / 10;
            if (sensors->gps_secondary.homedir) sensor.homedir = *sensors->gps_secondary.homedir * 10;
            memcpy(&buffer[3], &sensor, sizeof(ghst_sensor_gps_secondary_formatted_t));
            buffer[GHST_PACKET_LEN + 1] = get_crc(&buffer[2], GHST_PACKET_LEN + 1);
            break;
        }
        case TYPE_MAGBARO: {
            buffer[2] = GHST_DL_MAGBARO;
            ghst_sensor_magbaro_formatted_t sensor = {0};
            if (sensors->magbaro.vario) sensor.vario = *sensors->magbaro.vario * 100;
            memcpy(&buffer[3], &sensor, sizeof(ghst_sensor_magbaro_formatted_t));
            buffer[GHST_PACKET_LEN + 1] = get_crc(&buffer[2], GHST_PACKET_LEN + 1);
            break;
        }
    }
}

static inline uint sensor_count(ghst_sensors_t *sensors) {
    uint count = 0;
    for (uint i = 0; i < MAX_SENSORS; i++)
        if (sensors->enabled_sensors[i]) count++;
    return count;
}

static void send_packet(ghst_sensors_t *sensors) {
    if (!sensor_count(sensors)) return;
    static uint type = 0;
    uint8_t buffer[GHST_PACKET_LEN + 2] = {0};
    while (!sensors->enabled_sensors[type % MAX_SENSORS]) type++;
    format_sensor(sensors, type % MAX_SENSORS, buffer);
    uart0_write_bytes(buffer, GHST_PACKET_LEN + 2);
    type++;
    debug("\nGHST (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(buffer, GHST_PACKET_LEN + 2, "0x%X ");

    // blink led
    vTaskResume(context.led_task_handle);
}

static uint8_t get_crc(const uint8_t *ptr, uint32_t len) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc = crc8(crc, *ptr++);
    }
    return crc;
}

static uint8_t crc8(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

static void set_config(ghst_sensors_t *sensors) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    if (config->esc_protocol == ESC_APD_F) {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                            config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensors->enabled_sensors[TYPE_PACK_STAT] = true;
        sensors->pack_stat.voltage = parameter.voltage;
        sensors->pack_stat.current = parameter.current;
        sensors->pack_stat.consumed = parameter.consumption;

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

        sensors->enabled_sensors[TYPE_GPS_PRIMARY] = true;
        sensors->gps_primary.latitude = parameter.lat;
        sensors->gps_primary.longitude = parameter.lon;
        sensors->gps_primary.altitude = parameter.alt;
        sensors->enabled_sensors[TYPE_GPS_SECONDARY] = true;
        sensors->gps_secondary.groundspeed = parameter.spd_kmh;
        sensors->gps_secondary.heading = parameter.cog;
        sensors->gps_secondary.satellites = parameter.sat;
        sensors->gps_secondary.distance = parameter.dist;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);

        sensors->enabled_sensors[TYPE_PACK_STAT] = true;
        sensors->pack_stat.voltage = parameter.voltage;

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

        sensors->enabled_sensors[TYPE_PACK_STAT] = true;
        sensors->pack_stat.current = parameter.current;
        sensors->pack_stat.consumed = parameter.consumption;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);

        sensors->enabled_sensors[TYPE_MAGBARO] = true;
        sensors->magbaro.altitude = parameter.altitude;
        sensors->magbaro.vario = parameter.vspeed;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);

        sensors->enabled_sensors[TYPE_MAGBARO] = true;
        sensors->magbaro.altitude = parameter.altitude;
        sensors->magbaro.vario = parameter.vspeed;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);

        sensors->enabled_sensors[TYPE_MAGBARO] = true;
        sensors->magbaro.altitude = parameter.altitude;
        sensors->magbaro.vario = parameter.vspeed;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}