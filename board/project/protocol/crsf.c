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
    uart0_begin(420000L, UART_RECEIVER_TX, UART_RECEIVER_RX, CRSF_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    debug("\nCRSF init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static void process(void) {
    uint len = uart0_available();
    if (len >= 4 && len <= 64) {
        uint8_t buffer[len];
        uart0_read_bytes(buffer, len);
        debug("\nCRSF (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, len, "0x%X ");
        if (buffer[0] == 0xC8 && get_crc(&buffer[2], len - 3) == buffer[len - 1])
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

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
const unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D,
    0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F,
    0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF,
    0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E,
    0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67,
    0xB2, 0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE,
    0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45, 0x11,
    0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D,
    0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B,
    0xB1, 0x64, 0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20,
    0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05,
    0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

static uint8_t get_crc2(const uint8_t *ptr, uint32_t len) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}