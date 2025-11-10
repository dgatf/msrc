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
#include "esc_omp_m4.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "gps.h"
#include "ibus.h"
#include "ina3221.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

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
#define CRSF_FRAMETYPE_AIRSPEED 0x0A
#define CRSF_FRAMETYPE_RPM 0x0C
#define CRSF_FRAMETYPE_TEMP 0x0D
#define CRSF_FRAMETYPE_VOLTAGES 0x0E
#define CRSF_FRAMETYPE_GPS_TIME 0x03
#define CRSF_FRAMETYPE_GPS_EXTENDED 0x06

#define CRSF_TIMEOUT_US 1000

typedef enum crsf_sensor_type_t {
    TYPE_GPS,
    TYPE_VARIO,
    TYPE_BATERY,
    TYPE_BARO,
    TYPE_AIRSPEED,
    TYPE_RPM,
    TYPE_TEMP,
    TYPE_VOLTAGES,
    TYPE_GPS_TIME,
    TYPE_GPS_EXTENDED,
    TYPE_ATTITUDE,
    MAX_SENSORS
} crsf_sensor_type_t;

/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/

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

typedef struct crsf_sensor_airspeed_formatted_t {
    uint16_t speed;  // V * 10 big endian
} __attribute__((packed)) crsf_sensor_airspeed_formatted_t;

typedef struct crsf_sensor_rpm_formatted_t {
    uint8_t source;
    uint32_t rpm : 24;
} __attribute__((packed)) crsf_sensor_rpm_formatted_t;

typedef struct crsf_sensor_temp_formatted_t {
    uint8_t source;
    int16_t temp;
} __attribute__((packed)) crsf_sensor_temp_formatted_t;

typedef struct crsf_sensor_voltages_formatted_t {
    uint8_t source;
    uint16_t voltage;
} __attribute__((packed)) crsf_sensor_voltages_formatted_t;

typedef struct crsf_sensor_gps_time_formatted_t {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} __attribute__((packed)) crsf_sensor_gps_time_formatted_t;

typedef struct crsf_sensor_gps_extended_formatted_t {
    uint8_t fix_type;       // Current GPS fix quality
    int16_t n_speed;        // Northward (north = positive) Speed [cm/sec]
    int16_t e_speed;        // Eastward (east = positive) Speed [cm/sec]
    int16_t v_speed;        // Vertical (up = positive) Speed [cm/sec]
    int16_t h_speed_acc;    // Horizontal Speed accuracy cm/sec
    int16_t track_acc;      // Heading accuracy in degrees scaled with 1e-1 degrees times 10)
    int16_t alt_ellipsoid;  // Meters Height above GPS Ellipsoid (not MSL)
    int16_t h_acc;          // horizontal accuracy in cm
    int16_t v_acc;          // vertical accuracy in cm
    uint8_t reserved;
    uint8_t hDOP;  // Horizontal dilution of precision,Dimensionless in nits of.1.
    uint8_t vDOP;  // vertical dilution of precision, Dimensionless in nits of .1.
} __attribute__((packed)) crsf_sensor_gps_extended_formatted_t;

typedef struct crsf_sensor_gps_t {
    float *latitude;     // degree / 10,000,000 big endian
    float *longitude;    // degree / 10,000,000 big endian
    float *groundspeed;  // km/h / 10 big endian
    float *heading;      // GPS heading, degree/100 big endian
    float *altitude;     // meters, +1000m big endian
    float *satellites;   // satellites
} crsf_sensor_gps_t;

typedef struct crsf_sensor_attitude_formatted_t {
    int16_t pitch;  // angle ( rad / 10000 )
    int16_t roll;   // angle ( rad / 10000 )
    int16_t yaw;    // angle ( rad / 10000 )
} __attribute__((packed)) crsf_sensor_attitude_formatted_t;

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

typedef struct crsf_sensor_airspeed_t {
    float *speed;  // km/h * 10 big endian
} crsf_sensor_airspeed_t;

typedef struct crsf_sensor_rpm_t {
    float *rpm;
} crsf_sensor_rpm_t;

typedef struct crsf_sensor_temp_t {
    uint8_t temperature_count;
    float *temperature[5];  // V * 10 big endian
} crsf_sensor_temp_t;

typedef struct crsf_sensor_voltages_t {
    bool is_cell_average;
    uint8_t *cell_count;
    float *cell[18];
    uint8_t voltage_count;
    float *voltage[10];
} crsf_sensor_voltages_t;

typedef struct crsf_sensor_gps_time_t {
    float *date;
    float *time;
} crsf_sensor_gps_time_t;

typedef struct crsf_sensor_gps_extended_t {
    float *fix;
    float *n_speed;
    float *e_speed;
    float *v_speed;
    float *h_speed_acc;
    float *track_acc;
    float *alt_ellipsoid;
    float *h_acc;
    float *v_acc;
    float *hdop;
    float *vdop;
} crsf_sensor_gps_extended_t;

typedef struct crsf_sensor_attitude_t {
    float *pitch;
    float *roll;
    float *yaw;
} crsf_sensor_attitude_t;

typedef struct crsf_sensors_t {
    bool enabled_sensors[MAX_SENSORS];
    crsf_sensor_gps_t gps;
    crsf_sensor_vario_t vario;
    crsf_sensor_battery_t battery;
    crsf_sensor_baro_t baro;
    crsf_sensor_airspeed_t airspeed;
    crsf_sensor_rpm_t rpm;
    crsf_sensor_temp_t temperature;
    crsf_sensor_voltages_t voltages;
    crsf_sensor_gps_time_t gps_time;
    crsf_sensor_gps_extended_t gps_extended;
    crsf_sensor_attitude_t attitude;
} crsf_sensors_t;

static uint8_t format_sensor(crsf_sensors_t *sensors, uint8_t type, uint8_t *buffer);
static void send_packet(crsf_sensors_t *sensors);
static uint8_t get_crc(const uint8_t *ptr, uint32_t len);
static uint8_t crc8(uint8_t crc, unsigned char a);
static void set_config(crsf_sensors_t *sensors);
static inline uint sensor_count(crsf_sensors_t *sensors);

void crsf_task(void *parameters) {
    crsf_sensors_t sensors = {0};
    set_config(&sensors);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(416666L, UART_RECEIVER_TX, UART_RECEIVER_RX, CRSF_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);
    debug("\nCRSF init");
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        send_packet(&sensors);
    }
}

static uint8_t format_sensor(crsf_sensors_t *sensors, uint8_t type, uint8_t *buffer) {
    // Packet format: [sync] [len (from type)] [type] [payload] [crc8 from type]
    uint len = 0;
    buffer[0] = 0xC8;
    switch (type) {
        case TYPE_GPS: {
            buffer[1] = sizeof(crsf_sensor_gps_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_GPS;
            crsf_sensor_gps_formatted_t sensor = {0};
            if (sensors->gps.latitude) sensor.latitude = swap_32((int32_t)(*sensors->gps.latitude * 10000000L));
            if (sensors->gps.longitude) sensor.longitude = swap_32((int32_t)(*sensors->gps.longitude * 10000000L));
            if (sensors->gps.groundspeed)
                sensor.groundspeed = swap_16((uint16_t)(fabs(*sensors->gps.groundspeed * 10)));
            if (sensors->gps.satellites) sensor.satellites = *sensors->gps.satellites;
            if (sensors->gps.heading) sensor.heading = swap_16((uint16_t)(*sensors->gps.heading * 100));
            if (sensors->gps.altitude) {
                float altitude = *sensors->gps.altitude + 1000;
                if (altitude < 0) altitude = 0;
                if (altitude > 65535) altitude = 65535;
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
                if (altitude > 3276) altitude = 3276;
                sensor.altitude = swap_16((uint16_t)(altitude * 10));
            }
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_baro_formatted_t));
            buffer[3 + sizeof(crsf_sensor_baro_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_baro_formatted_t) + 1);
            len = sizeof(crsf_sensor_baro_formatted_t) + 4;
            break;
        }
        case TYPE_AIRSPEED: {
            buffer[1] = sizeof(crsf_sensor_airspeed_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_AIRSPEED;
            crsf_sensor_airspeed_formatted_t sensor = {0};
            if (sensors->airspeed.speed) sensor.speed = swap_16((int16_t)(*sensors->airspeed.speed * 10));
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_airspeed_formatted_t));
            buffer[3 + sizeof(crsf_sensor_airspeed_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_airspeed_formatted_t) + 1);
            len = sizeof(crsf_sensor_airspeed_formatted_t) + 4;
            break;
        }
        case TYPE_RPM: {
            buffer[1] = sizeof(crsf_sensor_rpm_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_RPM;
            crsf_sensor_rpm_formatted_t sensor = {0};
            if (sensors->rpm.rpm) sensor.rpm = swap_24((int32_t)(*sensors->rpm.rpm));
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_rpm_formatted_t));
            buffer[3 + sizeof(crsf_sensor_rpm_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_rpm_formatted_t) + 1);
            len = sizeof(crsf_sensor_rpm_formatted_t) + 4;
            break;
        }
        case TYPE_TEMP: {
            static uint8_t count = 0;
            buffer[1] = sizeof(crsf_sensor_temp_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_TEMP;
            crsf_sensor_temp_formatted_t sensor = {0};
            sensor.source = count;
            if (sensors->temperature.temperature[count])
                sensor.temp = swap_16((int16_t)(*sensors->temperature.temperature[count] * 10));
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_temp_formatted_t));
            buffer[3 + sizeof(crsf_sensor_temp_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_temp_formatted_t) + 1);
            len = sizeof(crsf_sensor_temp_formatted_t) + 4;
            count++;
            if (count >= sensors->temperature.temperature_count) count = 0;
            break;
        }
        case TYPE_VOLTAGES: {
            static uint8_t cell_index = 0;
            static uint8_t voltage_index = 0;
            static bool send_cell = true;
            buffer[2] = CRSF_FRAMETYPE_VOLTAGES;
            crsf_sensor_voltages_formatted_t sensor = {0};
            buffer[1] = sizeof(crsf_sensor_voltages_formatted_t) + 2;
            if (send_cell && *sensors->voltages.cell_count > 0) {
                sensor.source = cell_index;
                if (sensors->voltages.is_cell_average && sensors->voltages.cell[0]) {
                    sensor.voltage = swap_16((uint16_t)(*sensors->voltages.cell[0] * 1000));
                } else if (sensors->voltages.cell[cell_index]) {
                    sensor.voltage = swap_16((uint16_t)(*sensors->voltages.cell[cell_index] * 1000));
                }

                memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_voltages_formatted_t));
                buffer[3 + sizeof(crsf_sensor_voltages_formatted_t)] =
                    get_crc(&buffer[2], sizeof(crsf_sensor_voltages_formatted_t) + 1);
                len = sizeof(crsf_sensor_voltages_formatted_t) + 4;
                buffer[1] = sizeof(crsf_sensor_voltages_formatted_t) + 2;

                cell_index++;
                if (cell_index >= *sensors->voltages.cell_count) {
                    cell_index = 0;
                    send_cell = false;
                }
            } else if (sensors->voltages.voltage_count > 0) {
                sensor.source = voltage_index + 128;
                sensor.voltage = swap_16((uint16_t)(*sensors->voltages.voltage[voltage_index] * 1000));

                memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_voltages_formatted_t));
                buffer[3 + sizeof(crsf_sensor_voltages_formatted_t)] =
                    get_crc(&buffer[2], sizeof(crsf_sensor_voltages_formatted_t) + 1);
                len = sizeof(crsf_sensor_voltages_formatted_t) + 4;
                buffer[1] = sizeof(crsf_sensor_voltages_formatted_t) + 2;

                voltage_index++;
                if (voltage_index >= sensors->voltages.voltage_count) {
                    voltage_index = 0;
                    send_cell = true;
                }
            }
            break;
        }
        case TYPE_GPS_TIME: {
            buffer[1] = sizeof(crsf_sensor_gps_time_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_GPS_TIME;
            crsf_sensor_gps_time_formatted_t sensor = {0};
            uint16_t year = *sensors->gps_time.date / 10000 + 2000;
            sensor.year = swap_16(year);
            sensor.month = ((uint)*sensors->gps_time.date - (year - 2000) * 10000) / 100;
            sensor.day = (uint)*sensors->gps_time.date - (year - 2000) * 10000 - sensor.month * 100;
            sensor.hour = (uint)*sensors->gps_time.time / 10000;
            sensor.minute = ((uint)*sensors->gps_time.time - sensor.hour * 10000) / 100;
            sensor.second = ((uint)*sensors->gps_time.time - sensor.hour * 10000 - sensor.minute * 100);
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_gps_time_formatted_t));
            buffer[3 + sizeof(crsf_sensor_gps_time_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_gps_time_formatted_t) + 1);
            len = sizeof(crsf_sensor_gps_time_formatted_t) + 4;
            break;
        }
        case TYPE_GPS_EXTENDED: {
            buffer[1] = sizeof(crsf_sensor_gps_extended_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_GPS_EXTENDED;
            crsf_sensor_gps_extended_formatted_t sensor = {0};
            sensor.fix_type = *sensors->gps_extended.fix;
            sensor.n_speed = *sensors->gps_extended.n_speed / 10;
            sensor.e_speed = *sensors->gps_extended.e_speed / 10;
            sensor.v_speed = *sensors->gps_extended.v_speed / 10;
            sensor.h_speed_acc = *sensors->gps_extended.h_speed_acc / 10;
            sensor.track_acc = *sensors->gps_extended.track_acc / 10;
            sensor.alt_ellipsoid = *sensors->gps_extended.alt_ellipsoid / 10;
            sensor.hDOP = *sensors->gps_extended.hdop * 10;
            sensor.vDOP = *sensors->gps_extended.vdop * 10;
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_gps_extended_formatted_t));
            buffer[3 + sizeof(crsf_sensor_gps_extended_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_gps_extended_formatted_t) + 1);
            len = sizeof(crsf_sensor_gps_extended_formatted_t) + 4;
            break;
        }
        case TYPE_ATTITUDE: {
            buffer[1] = sizeof(crsf_sensor_attitude_formatted_t) + 2;
            buffer[2] = CRSF_FRAMETYPE_ATTITUDE;
            crsf_sensor_attitude_formatted_t sensor = {0};
            if (sensors->attitude.pitch) sensor.pitch = swap_16((int16_t)(*sensors->attitude.pitch * 10000));
            if (sensors->attitude.roll) sensor.roll = swap_16((int16_t)(*sensors->attitude.roll * 10000));
            if (sensors->attitude.yaw) sensor.yaw = swap_16((int16_t)(*sensors->attitude.yaw * 10000));
            memcpy(&buffer[3], &sensor, sizeof(crsf_sensor_attitude_formatted_t));
            buffer[3 + sizeof(crsf_sensor_attitude_formatted_t)] =
                get_crc(&buffer[2], sizeof(crsf_sensor_attitude_formatted_t) + 1);
            len = sizeof(crsf_sensor_attitude_formatted_t) + 4;
            break;
        }
    }
    return len;
}

static inline uint sensor_count(crsf_sensors_t *sensors) {
    uint count = 0;
    for (uint i = 0; i < MAX_SENSORS; i++)
        if (sensors->enabled_sensors[i]) count++;
    return count;
}

static void send_packet(crsf_sensors_t *sensors) {
    if (!sensor_count(sensors)) return;
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
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_fet;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_bec;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;
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

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_fet;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_bec;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.voltage_bec;
        sensors->voltages.voltage_count++;
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

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.voltage_bec;
        sensors->voltages.voltage_count++;
        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.ripple_voltage;
        sensors->voltages.voltage_count++;

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

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_fet;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_bec;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.voltage_bec;
        sensors->voltages.voltage_count++;

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

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

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

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_fet;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_bec;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temperature_bat;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = false;
        sensors->voltages.cell_count = parameter.cells;
        for (uint i = 0; i < 18; i++) sensors->voltages.cell[i] = parameter.cell[i];

        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.voltage_bec;
        sensors->voltages.voltage_count++;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temp_esc;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temp_motor;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

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

        sensors->enabled_sensors[TYPE_BATERY] = true;
        sensors->battery.voltage = parameter.voltage;
        sensors->battery.current = parameter.current;
        sensors->battery.capacity = parameter.consumption;

        sensors->enabled_sensors[TYPE_RPM] = true;
        sensors->rpm.rpm = parameter.rpm;

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temp_esc;
        sensors->temperature.temperature_count++;

        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.temp_motor;
        sensors->temperature.temperature_count++;

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.is_cell_average = true;
        sensors->voltages.cell_count = parameter.cell_count;
        sensors->voltages.cell[0] = parameter.cell_voltage;

        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.bec_voltage;
        sensors->voltages.voltage_count++;

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

        sensors->enabled_sensors[TYPE_GPS] = true;
        sensors->gps.latitude = parameter.lat;
        sensors->gps.longitude = parameter.lon;
        sensors->gps.groundspeed = parameter.spd_kmh;
        sensors->gps.heading = parameter.cog;
        sensors->gps.satellites = parameter.sat;
        sensors->gps.altitude = parameter.alt;

        /*sensors->enabled_sensors[TYPE_GPS_TIME] = true;
        sensors->gps_time.date = parameter.date;
        sensors->gps_time.time = parameter.time;

        sensors->enabled_sensors[TYPE_GPS_EXTENDED] = true;
        sensors->gps_extended.hdop = parameter.hdop;
        sensors->gps_extended.fix = parameter.fix;
        sensors->gps_extended.vdop = parameter.vdop;
        sensors->gps_extended.n_speed = parameter.n_vel;
        sensors->gps_extended.e_speed = parameter.e_vel;
        sensors->gps_extended.v_speed = parameter.v_vel;
        sensors->gps_extended.h_speed_acc = parameter.h_acc;
        sensors->gps_extended.track_acc = parameter.track_acc;
        sensors->gps_extended.alt_ellipsoid = parameter.alt_elipsiod;
        sensors->gps_extended.h_acc = parameter.h_acc;
        sensors->gps_extended.v_acc = parameter.v_acc;*/

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        sensors->voltages.voltage[sensors->voltages.voltage_count] = parameter.voltage;
        sensors->voltages.voltage_count++;

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
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->enabled_sensors[TYPE_TEMP] = true;
        sensors->temperature.temperature[sensors->temperature.temperature_count] = parameter.ntc;
        sensors->temperature.temperature_count++;

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

        sensors->enabled_sensors[TYPE_BARO] = true;
        sensors->baro.altitude = parameter.altitude;
        sensors->baro.vspeed = parameter.vspeed;

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

        sensors->enabled_sensors[TYPE_BARO] = true;
        sensors->baro.altitude = parameter.altitude;
        sensors->baro.vspeed = parameter.vspeed;

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

        sensors->enabled_sensors[TYPE_BARO] = true;
        sensors->baro.altitude = parameter.altitude;
        sensors->baro.vspeed = parameter.vspeed;

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

        sensors->enabled_sensors[TYPE_AIRSPEED] = true;
        sensors->airspeed.speed = parameter.airspeed;

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

        sensors->enabled_sensors[TYPE_ATTITUDE] = true;
        sensors->attitude.pitch = parameter.pitch;
        sensors->attitude.roll = parameter.roll;
        sensors->attitude.yaw = parameter.yaw;

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

        sensors->enabled_sensors[TYPE_VOLTAGES] = true;
        *sensors->voltages.cell_count = parameter.cell_count;
        for (uint i = 0; i < parameter.cell_count; i++) {
            sensors->voltages.cell[i] = parameter.cell[i];
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}