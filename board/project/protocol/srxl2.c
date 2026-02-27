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
#include "esc_omp_m4.h"
#include "esc_openyge.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gps.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ina3221.h"
#include "mpu6050.h"
#include "ms5611.h"
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
#define SRXL2_DEVICE_BAUDRATE 1  // 0 = 115200. 1 = 400000
#define SRXL2_DEVICE_INFO 0
#define SRXL2_DEVICE_UID 0x12345678
#define MAX_BAD_FRAMES 50

static volatile uint8_t dest_id = 0xFF;
static volatile uint8_t baudrate = 0;
static alarm_id_t alarm_id;
static volatile bool send_handshake = false;

static void process(void);
static void send_packet(void);
static void set_config(void);
static int64_t alarm_50ms(alarm_id_t id, void *user_data);

void srxl2_task(void *parameters) {
    alarm_id = add_alarm_in_us(50000, alarm_50ms, NULL, true);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    uart0_begin(115200, UART_RECEIVER_TX, UART_RECEIVER_RX, SRXL2_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    xbus_set_config();
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

        // Check for bad frames
        static uint bad_frames = 0;
        if (bad_frames > MAX_BAD_FRAMES) {
            if (baudrate) {
                uart_set_baudrate(uart0, 115200);
                baudrate = 0;
                debug("\nSRXL2. Autobaud 115200");
            } else {
                uart_set_baudrate(uart0, 400000);
                baudrate = 1;
                debug("\nSRXL2. Autobaud 400000");
            }
            bad_frames = 0;
            return;
        }

        if (length < 3 || length > 128) {
            if (length == 1) return;
            debug("\nSRXL2. Bad Fr %u (len %u)", bad_frames, length);
            bad_frames++;
            return;
        }
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
            // Allow packets with wrong crc for handshake
            if (dest_id != 0xFF ||
                !(data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[4] == SRXL2_DEVICE_ID)) {
                bad_frames++;
                return;
            }
        }
        bad_frames = 0;

        // Handshake received
        if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[4] == SRXL2_DEVICE_ID) {
            dest_id = data[3];
            debug("\nSRXL2. Set dest_id 0x%X", dest_id);
            srxl2_send_handshake(uart0, SRXL2_DEVICE_ID, dest_id, SRXL2_DEVICE_PRIORITY, SRXL2_DEVICE_BAUDRATE,
                                 SRXL2_DEVICE_INFO, SRXL2_DEVICE_UID);

        }
        // Send telemetry
        else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_CONTROL && data[4] == SRXL2_DEVICE_ID) {
            send_packet();
        }
        // Set baudrate
        else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[4] == 0xFF) {
            baudrate = data[6];
            if (baudrate)
                uart_set_baudrate(uart0, 400000);
            else
                uart_set_baudrate(uart0, 115200);
            debug("\nSRXL2. Set baudrate %u", baudrate);
        }
    }
}

static void send_packet(void) {
    static uint cont = 0;
    if (!srxl_sensors_count()) return;
    while (!sensor.is_enabled[cont]) {
        cont++;
        if (cont > XBUS_STRU_TELE_DIGITAL_AIR) cont = 0;
    }
    srxl2_telemetry_t packet = {0};
    packet.header = SRXL2_HEADER;
    packet.type = SRXL2_PACKET_TYPE_TELEMETRY;
    packet.len = SRXL2_TELEMETRY_LEN;
    packet.dest_id = dest_id;
    uint8_t buffer[16];
    switch (cont) {
        case XBUS_AIRSPEED:
            xbus_format_sensor(XBUS_AIRSPEED_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_airspeed_t));
            break;
        case XBUS_BATTERY:
            xbus_format_sensor(XBUS_BATTERY_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_battery_t));
            break;
        case XBUS_ESC:
            xbus_format_sensor(XBUS_ESC_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_esc_t));
            break;
        case XBUS_GPS_LOC:
            xbus_format_sensor(XBUS_GPS_LOC_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_gps_loc_t));
            break;
        case XBUS_GPS_STAT:
            xbus_format_sensor(XBUS_GPS_STAT_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_gps_stat_t));
            break;
        case XBUS_RPMVOLTTEMP:
            xbus_format_sensor(XBUS_RPMVOLTTEMP_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_rpm_volt_temp_t));
            break;
        case XBUS_FUEL_FLOW:
            xbus_format_sensor(XBUS_FUEL_FLOW_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, (uint8_t *)buffer, sizeof(xbus_fuel_flow_t));
            break;
        case XBUS_STRU_TELE_DIGITAL_AIR:
            xbus_format_sensor(XBUS_STRU_TELE_DIGITAL_AIR_ID, buffer);
            memcpy((uint8_t *)&packet.xbus_packet, buffer, sizeof(xbus_stru_tele_digital_air_t));
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
    if (baudrate) uart_set_baudrate(uart0, 115200);
    baudrate = 0;
    vTaskNotifyGiveIndexedFromISR(context.uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 50000;
}
