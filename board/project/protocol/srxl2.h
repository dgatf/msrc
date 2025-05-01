#ifndef SRXL2_H
#define SRXL2_H

#include "common.h"
#include "xbus.h"

#define SRXL2_TIMEOUT_US 1000
#define SRXL2_HEADER 0xA6
#define SRXL2_HANDSHAKE_LEN 14
#define SRXL2_TELEMETRY_LEN 22
#define SRXL2_CONTROL_CMD_CHANNEL 0x00       // channel data
#define SRXL2_PACKET_TYPE_HANDSHAKE 0x21
#define SRXL2_PACKET_TYPE_CONTROL 0xCD
#define SRXL2_PACKET_TYPE_TELEMETRY 0x80

extern context_t context;
extern xbus_sensor_t *sensor;
extern xbus_sensor_formatted_t *sensor_formatted;

typedef struct srxl2_handshake_t {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t source_id;
    uint8_t dest_id;
    uint8_t priority;
    uint8_t baudrate;
    uint8_t info;
    uint32_t uid;
    uint16_t crc;
} __attribute__((packed)) srxl2_handshake_t;

typedef struct srxl2_telemetry_t {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t dest_id;
    uint8_t xbus_packet[16];
    uint16_t crc;
} __attribute__((packed)) srxl2_telemetry_t;

void srxl2_task(void *parameters);
void srxl2_send_handshake(uart_inst_t *uart, uint8_t source_id, uint8_t dest_id, uint8_t priority, uint8_t baudrate,
                          uint8_t info, uint uid);

#endif