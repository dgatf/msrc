#include "esc_xdfly.h"

#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

// Big endian
typedef struct esc_xdfly_telemetry_packet_t {
    uint8_t header;
    uint8_t version;
    uint8_t length;
    uint16_t rpm;
    uint16_t temp;
    uint16_t throttle;
    uint16_t power;
    uint16_t voltage;
    uint16_t current;
    uint16_t consumption;
    uint16_t status;
    uint16_t bec_voltage;
    uint16_t crc;
} __attribute((packed)) esc_xdfly_telemetry_packet_t;

typedef struct esc_xdfly_request_packet_t {
    uint8_t header;
    uint8_t len;
    uint8_t cmd;
    uint8_t parameter;
    uint16_t parameter_value;
    uint16_t crc;
} __attribute((packed)) esc_xdfly_request_packet_t;

#define XDFLY_TIMEOUT_US 1000
#define XDFLY_CMD_PACKET_LENGHT sizeof(esc_xdfly_request_packet_t)
#define XDFLY_TELEMETRY_PACKET_LENGHT sizeof(esc_xdfly_telemetry_packet_t)
#define XDFLY_CMD_PACKET_HEADER 0xA5
#define XDFLY_TELEMETRY_PACKET_HEADER 0xDD
#define XDFLY_CMD_HANDSHAKE 0x03

static void process(esc_xdfly_parameters_t *parameter);
static void send_handshake(void);
static uint16_t get_crc16_modbus(const uint8_t *ptr, size_t len);

void esc_xdfly_task(void *parameters) {
    esc_xdfly_parameters_t parameter = *(esc_xdfly_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.temp = 0;
    *parameter.voltage = 0;
    *parameter.bec_voltage = 0;
    *parameter.current = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.cell_count = 1;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.temp = 12.34;
    *parameter.voltage = 12.34;
    *parameter.bec_voltage = 4.56;
    *parameter.current = 12.34;
    *parameter.consumption = 12.34;
    *parameter.rpm = 12345.67;
    *parameter.cell_voltage = 3.75;
#endif

    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, XDFLY_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);

    send_handshake();

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(esc_xdfly_parameters_t *parameter) {
    static uint32_t timestamp = 0;
    uint8_t length = uart1_available();
    if (length == XDFLY_TELEMETRY_PACKET_LENGHT) {
        esc_xdfly_telemetry_packet_t packet;
        uart1_read_bytes((uint8_t *)&packet, XDFLY_TELEMETRY_PACKET_LENGHT);
        debug("\nXDFLY (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(&packet, length, "0x%X ");
        if (packet.header != XDFLY_TELEMETRY_PACKET_HEADER || packet.length != XDFLY_TELEMETRY_PACKET_LENGHT) return;
        float rpm = swap_16(packet.rpm) / 10.0;
        float temp = swap_16(packet.temp) / 10.0;
        float voltage = swap_16(packet.voltage) / 100.0;
        float bec_voltage = swap_16(packet.bec_voltage) / 1000.0;
        float current = swap_16(packet.current) / 100.0;
        float consumption = swap_16(packet.consumption);
        rpm *= parameter->rpm_multiplier;
        *parameter->temp = get_average(parameter->alpha_temperature, *parameter->temp, temp);
        *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
        *parameter->bec_voltage = get_average(parameter->alpha_voltage, *parameter->bec_voltage, bec_voltage);
        *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
        *parameter->consumption = get_average(parameter->alpha_voltage, *parameter->consumption, consumption);
        *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
        *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
        debug(
            "\nXDFLY (%u) < Rpm: %.0f Volt: %.1f Curr: %.1f Volt BEC: %.1f Temp: %.0f Cons: %.0f "
            "CellV: %.2f",
            uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, *parameter->current,
            *parameter->bec_voltage, *parameter->temp, *parameter->consumption, *parameter->cell_voltage);
    } else {
        uint8_t buffer[length];
        uart1_read_bytes(buffer, length);
        debug("\nXDFLY (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, length, "0x%X ");

    }
}

static void send_handshake(void) {
    esc_xdfly_request_packet_t packet = {0};
    packet.header = XDFLY_CMD_PACKET_HEADER;
    packet.len = XDFLY_CMD_PACKET_LENGHT;
    packet.crc = swap_16(get_crc16_modbus((uint8_t *)&packet, XDFLY_CMD_PACKET_LENGHT - 2));
    uart1_write_bytes((uint8_t *)&packet, XDFLY_CMD_PACKET_LENGHT);
    debug("\nXDFLY (%u). Send Handshake > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(&packet, sizeof(esc_xdfly_request_packet_t), "0x%X ");
}

static uint16_t get_crc16_modbus(const uint8_t *ptr, size_t len) {
    uint16_t crc = ~0;
    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}
