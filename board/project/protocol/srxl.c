#include "srxl.h"

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
#include "esc_omp_m4.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gps.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "xgzp68xxd.h"

#define SRXL_HEADER 0xA5
#define SRXL_TELEMETRY_LEN 21
#define SRXL_FRAME_LEN 18
#define SRXL_PACKET_TYPE_TELEMETRY 0x80
#define SRXL_TIMEOUT_US 1000

typedef struct srxl_telemetry_t {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t xbus_packet[16];
    uint16_t crc;
} __attribute__((packed)) srxl_telemetry_t;

static void process(void);
static void send_packet(void);

void srxl_task(void *parameters) {
    sensors = calloc(1, sizeof(xbus_sensors_t));

    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    uart0_begin(115200, UART_RECEIVER_TX, UART_RECEIVER_RX, SRXL_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    xbus_set_config();
    debug("\nSRXL init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

uint16_t srxl_get_crc(uint8_t *buffer, uint8_t length) {
    uint16_t crc = 0;
    for (uint8_t i = 0; i < length; ++i) {
        crc = srxl_crc16(crc, buffer[i]);
    }
    return crc;
}

uint16_t srxl_crc16(uint16_t crc, uint8_t data) {
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc = crc << 1;
    }
    return crc;
}

uint srxl_sensors_count(xbus_sensors_t *sensors) {
    uint count = 0;
    for (uint i = 0; i <= XBUS_STRU_TELE_DIGITAL_AIR; i++) {
        if (sensors->is_enabled[i]) {
            count++;
            debug("\nc>%u", count);
        }
    }
    return count;
}

static void process(void) {
    static bool mute = true;
    uint8_t length = uart0_available();
    if (length) {
        uint8_t data[length];
        uart0_read_bytes(data, length);
        if (length == SRXL_FRAME_LEN) {
            // uint8_t data[SRXL_FRAME_LEN];
            if (data[0] == SRXL_HEADER) {
                debug("\nSRXL (%u) < ", uxTaskGetStackHighWaterMark(NULL));
                debug_buffer(data, SRXL_FRAME_LEN, "0x%X ");
                if (!mute) send_packet();
                mute = !mute;
            }
        }
    }
}

static void send_packet(void) {
    static uint cont = 0;
    if (!srxl_sensors_count(sensors)) return;
    while (!sensors->is_enabled[cont]) {
        cont++;
        if (cont > XBUS_ENERGY) cont = 0;
    }
    srxl_telemetry_t packet = {0};
    packet.header = SRXL_HEADER;
    packet.type = SRXL_PACKET_TYPE_TELEMETRY;
    packet.len = SRXL_TELEMETRY_LEN;
    uint8_t buffer[16] = {0};
    switch (cont) {
        case XBUS_AIRSPEED:
            xbus_format_sensor(XBUS_AIRSPEED_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_airspeed_t));
            break;
        case XBUS_BATTERY:
            xbus_format_sensor(XBUS_BATTERY_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_battery_t));
            break;
        case XBUS_ESC:
            xbus_format_sensor(XBUS_ESC_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_esc_t));
            break;
        case XBUS_GPS_LOC:
            xbus_format_sensor(XBUS_GPS_LOC_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_gps_loc_t));
            break;
        case XBUS_GPS_STAT:
            xbus_format_sensor(XBUS_GPS_STAT_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_gps_stat_t));
            break;
        case XBUS_RPMVOLTTEMP:
            xbus_format_sensor(XBUS_RPMVOLTTEMP_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_rpm_volt_temp_t));
            break;
        case XBUS_FUEL_FLOW:
            xbus_format_sensor(XBUS_FUEL_FLOW_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_fuel_flow_t));
            break;
        case XBUS_STRU_TELE_DIGITAL_AIR:
            xbus_format_sensor(XBUS_STRU_TELE_DIGITAL_AIR_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_stru_tele_digital_air_t));
            break;
    }
    uint16_t crc = srxl_get_crc(buffer, SRXL_TELEMETRY_LEN - 2);  // all bytes, including header
    packet.crc = swap_16(crc);
    uart0_write_bytes((uint8_t *)&packet, SRXL_TELEMETRY_LEN);
    cont++;
    debug("\nSRXL (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer((uint8_t *)&packet, SRXL_TELEMETRY_LEN, "0x%X ");
    vTaskResume(context.led_task_handle);
}