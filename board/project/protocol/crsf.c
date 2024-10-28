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

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)

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

#define IBUS_RECEIVED_NONE 0
#define IBUS_RECEIVED_POLL 1

#define IBUS_COMMAND_DISCOVER 0x8
#define IBUS_COMMAND_TYPE 0x9
#define IBUS_COMMAND_MEASURE 0xA

#define CRSF_TIMEOUT_US 1000
#define IBUS_PACKET_LENGHT 4

typedef struct crsf_sensor_gps_t {
    int32_t latitude;      // degree / 10,000,000 big endian
    int32_t longitude;     // degree / 10,000,000 big endian
    uint16_t groundspeed;  // km/h / 10 big endian
    uint16_t heading;      // GPS heading, degree/100 big endian
    uint16_t altitude;     // meters, +1000m big endian
    uint8_t satellites;    // satellites
} crsf_sensor_gps_t;

typedef struct crsf_sensor_vario_t {
    int16_t vspeed;  // cm/s
} crsf_sensor_vario_t;

typedef struct crsf_sensor_baro_t {
    uint16_t altitude;
    int16_t vspeed;  // cm/s
} crsf_sensor_baro_t;

typedef struct crsf_sensor_battery_t {
    uint16_t voltage;        // V * 10 big endian
    uint16_t current;        // A * 10 big endian
    uint32_t capacity : 24;  // used capacity mah big endian
    uint8_t remaining;       // %
} crsf_sensor_battery_t;

typedef struct crsf_sensors_t {
    crsf_sensor_gps_t gps;
    crsf_sensor_vario_t vario;
    crsf_sensor_battery_t battery;
    crsf_sensor_baro_t baro;
} crsf_sensors_t;

crsf_sensors_t sensors;

static void process(void);
static void send_packet(void);
static uint8_t crc8(uint8_t crc, unsigned char a);
static uint8_t get_crc(const uint8_t *ptr, uint32_t len);

void crsf_task(void *parameters) {
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(416666L, UART_RECEIVER_TX, UART_RECEIVER_RX, CRSF_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    debug("\nCRSF init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static void process(void) {
    uint len = uart0_available();
    if (len >= 4 && len <= 256) {
        uint8_t buffer[len];
        uart0_read_bytes(buffer, len);
        debug("\nCRSF (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, len, "0x%X ");
        if (/*buffer[0] == 0xC8 &&*/ get_crc(&buffer[2], len - 3) == buffer[len - 1])
            send_packet();
        else
            debug("\nCRSF. Bad CRC or header");
    }
}

static void send_packet(void) {
    // [sync] [len] [type] [payload] [crc8 from type] 

    uint8_t buffer[64] = {0};
    buffer[0] = 0xC8;
    buffer[1] = sizeof(sensors.vario) + 2;
    buffer[2] = CRSF_FRAMETYPE_VARIO;
    sensors.vario.vspeed = swap_16(0x0005);
    memcpy(&buffer[3], &sensors.vario, sizeof(sensors.vario));
    buffer[3 + sizeof(sensors.vario)] = get_crc(&buffer[2], sizeof(sensors.vario) + 1);
    uart0_write_bytes(buffer, sizeof(sensors.vario) + 4);
    debug("\nCRSF (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(buffer, sizeof(sensors.vario) + 4, "0x%X ");

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
