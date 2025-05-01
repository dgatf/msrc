#include "crsf.h"

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
#include "esc_pwm.h"
#include "ibus.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pwm_out.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "smart_esc.h"

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)
#define swap_24(value) (((value & 0xFF) << 16) | (value & 0xFF00) | (value & 0xFF0000) >> 16)
#define swap_32(value) \
    (((value & 0xFF) << 24) | ((value & 0xFF00) << 8) | ((value & 0xFF0000) >> 8) | ((value & 0xFF000000) >> 24))

#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_VARIO 0x07
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_BARO_ALTITUDE 0x09
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED 0x17
#define CRSF_FRAMETYPE_LINK_STATISTICS_RX 0x1C
#define CRSF_FRAMETYPE_LINK_STATISTICS_TX 0x1D
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21

#define CRSF_TIMEOUT_US 1000

#define TYPE_GPS 0
#define TYPE_VARIO 1
#define TYPE_BATERY 2
#define TYPE_BARO 3

#define MAX_SENSORS 4

typedef struct crsf_sensor_gps_formatted_t {
    int32_t latitude;      // degree / 10,000,000 big endian
    int32_t longitude;     // degree / 10,000,000 big endian
    uint16_t groundspeed;  // km/h / 10 big endian
    uint16_t heading;      // GPS heading, degree/100 big endian
    uint16_t altitude;     // meters, +1000m big endian
    uint8_t satellites;    // satellites
} __attribute__((packed)) crsf_sensor_gps_formatted_t;

typedef struct crsf_sensor_vario_formatted_t {
    int16_t vspeed;  // cm/s
} __attribute__((packed)) crsf_sensor_vario_formatted_t;

typedef struct crsf_sensor_baro_formatted_t {
    uint16_t altitude;
    int16_t vspeed;  // cm/s
} __attribute__((packed)) crsf_sensor_baro_formatted_t;

typedef struct crsf_sensor_battery_formatted_t {
    uint16_t voltage;        // V * 10 big endian
    uint16_t current;        // A * 10 big endian
    uint32_t capacity : 24;  // used capacity mah big endian
    uint8_t remaining;       // %
} __attribute__((packed)) crsf_sensor_battery_formatted_t;

typedef struct crsf_sensor_gps_t {
    float *latitude;     // degree / 10,000,000 big endian
    float *longitude;    // degree / 10,000,000 big endian
    float *groundspeed;  // km/h / 10 big endian
    float *heading;      // GPS heading, degree/100 big endian
    float *altitude;     // meters, +1000m big endian
    float *satellites;   // satellites
} crsf_sensor_gps_t;

typedef struct crsf_sensor_vario_t {
    float *vspeed;  // cm/s
} crsf_sensor_vario_t;

typedef struct crsf_sensor_baro_t {
    float *altitude;
    float *vspeed;  // cm/s
} crsf_sensor_baro_t;

typedef struct crsf_sensor_battery_t {
    float *voltage;    // V * 10 big endian
    float *current;    // A * 10 big endian
    float *capacity;   // used capacity mah big endian
    float *remaining;  // %
} crsf_sensor_battery_t;

typedef struct crsf_sensors_t {
    bool enabled_sensors[4];
    crsf_sensor_gps_t gps;
    crsf_sensor_vario_t vario;
    crsf_sensor_battery_t battery;
    crsf_sensor_baro_t baro;
} crsf_sensors_t;

static void process(crsf_sensors_t *sensors);
static uint8_t format_sensor(crsf_sensors_t *sensors, uint8_t type, uint8_t *buffer);
static void send_packet(crsf_sensors_t *sensors);
static uint8_t get_crc(const uint8_t *ptr, uint32_t len);
static uint8_t crc8(uint8_t crc, unsigned char a);
static void set_config(crsf_sensors_t *sensors);

void crsf_task(void *parameters) {
    crsf_sensors_t sensors = {0};
    set_config(&sensors);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(416666L, UART_RECEIVER_TX, UART_RECEIVER_RX, CRSF_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    debug("\nCRSF init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&sensors);
    }
}

static void process(crsf_sensors_t *sensors) {
    uint len = uart0_available();
    if (len >= 4 && len <= 256) {
        uint8_t buffer[len];
        uart0_read_bytes(buffer, len);
        debug("\nCRSF (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, len, "0x%X ");
        if (/*buffer[0] == 0xC8 &&*/ get_crc(&buffer[2], len - 3) == buffer[len - 1])
            send_packet(sensors);
        else
            debug("\nCRSF. Bad CRC or header");
    }
}

static uint8_t format_sensor(crsf_sensors_t *sensors, uint8_t type, uint8_t *buffer) {
    // Packet format: [sync] [len] [type] [payload] [crc8 from type]
    uint len = 0;
    buffer[0] = 0xC8;
    switch (type) {
        case TYPE_GPS: {
            buffer[1] = sizeof(crsf_sensor_gps_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_GPS;
            crsf_sensor_gps_formatted_t sensor = {0};
            if (sensors->gps.latitude) sensor.latitude = swap_32((int32_t)(*sensors->gps.latitude / 60 * 10000000L));
            if (sensors->gps.longitude) sensor.longitude = swap_32((int32_t)(*sensors->gps.longitude / 60 * 10000000L));
            if (sensors->gps.groundspeed) sensor.groundspeed = swap_16((uint16_t)(*sensors->gps.groundspeed * 10));
            if (sensors->gps.satellites) sensor.satellites = *sensors->gps.satellites;
            if (sensors->gps.heading) sensor.heading = swap_16((uint16_t)(*sensors->gps.heading * 100));
            if (sensors->gps.altitude) {
                float altitude = *sensors->gps.altitude + 1000;
                if (altitude < 0) altitude = 0;
                if (altitude > 2276) altitude = 2276;
                sensor.altitude = swap_16((uint16_t)altitude);
            }
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_gps_formatted_t));
            buffer[3 + sizeof(crsf_sensor_gps_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_gps_formatted_t) + 1);
            len = sizeof(crsf_sensor_gps_formatted_t) + 4;
            break;
        }
        case TYPE_VARIO: {
            buffer[1] = sizeof(crsf_sensor_vario_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_VARIO;
            crsf_sensor_vario_formatted_t sensor = {0};
            if (sensors->vario.vspeed) sensor.vspeed = swap_16((int16_t)(*sensors->vario.vspeed * 100));
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_vario_formatted_t));
            buffer[3 + sizeof(crsf_sensor_vario_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_vario_formatted_t) + 1);
            len = sizeof(crsf_sensor_vario_formatted_t) + 4;
            break;
        }
        case TYPE_BATERY: {
            buffer[1] = sizeof(crsf_sensor_battery_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
            crsf_sensor_battery_formatted_t sensor = {0};
            if (sensors->battery.voltage) sensor.voltage = swap_16((uint16_t)(*sensors->battery.voltage * 10));
            if (sensors->battery.current) sensor.current = swap_16((uint16_t)(*sensors->battery.current * 10));
            if (sensors->battery.capacity) sensor.capacity = swap_24((uint32_t)*sensors->battery.capacity);
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_battery_formatted_t));
            buffer[3 + sizeof(crsf_sensor_battery_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_battery_formatted_t) + 1);
            len = sizeof(crsf_sensor_battery_formatted_t) + 4;
            break;
        }
        case TYPE_BARO: {
            buffer[1] = sizeof(crsf_sensor_baro_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_BARO_ALTITUDE;
            crsf_sensor_baro_formatted_t sensor = {0};
            if (sensors->baro.vspeed) sensor.vspeed = swap_16((int16_t)(*sensors->baro.vspeed * 100));
            if (sensors->baro.altitude) {
                float altitude = *sensors->baro.altitude + 1000;
                if (altitude < 0) altitude = 0;
                if (altitude > 2276) altitude = 2276;
                sensor.altitude = swap_16((uint16_t)altitude);
            }
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_baro_formatted_t));
            buffer[3 + sizeof(crsf_sensor_baro_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_baro_formatted_t) + 1);
            len = sizeof(crsf_sensor_baro_formatted_t) + 4;
            break;
        }
    }
    return len;
}

static void send_packet(crsf_sensors_t *sensors) {
    if (!sensors->enabled_sensors[0] && !sensors->enabled_sensors[1] && !sensors->enabled_sensors[2] &&
        !sensors->enabled_sensors[3])
        return;
    static uint type = 0;
    uint8_t buffer[64] = {0};
    while (!sensors->enabled_sensors[type % MAX_SENSORS]) type++;
    uint len = format_sensor(sensors, type % MAX_SENSORS, buffer);
    uart0_write_bytes(buffer, len);
    type++;
    debug("\nCRSF (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(buffer, len, "0x%X ");

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

static void set_config(crsf_sensors_t *sensors) {
    config_t *config = config_read();
    TaskHandle_t task_handle;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;
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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;
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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

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

        sensors->enabled_sensors[TYPE_GPS] = true;
        sensors->gps.latitude = parameter.lat;
        sensors->gps.longitude = parameter.lon;
        sensors->gps.groundspeed = parameter.spd_kmh;
        sensors->gps.heading = parameter.cog;
        sensors->gps.satellites = parameter.sat;
        sensors->gps.altitude = parameter.alt;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    /*if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3, config->analog_rate, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->frame_0x1A[FRAME_0X1A_ASPD] = parameter.airspeed;
        sensor->is_enabled_frame[FRAME_0X1A] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }*/
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_BARO] = true;
        sensors->baro.altitude = parameter.altitude;
        sensors->baro.vspeed = parameter.vspeed;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_BARO] = true;
        sensors->baro.altitude = parameter.altitude;
        sensors->baro.vspeed = parameter.vspeed;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_BARO] = true;
        sensors->baro.altitude = parameter.altitude;
        sensors->baro.vspeed = parameter.vspeed;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
