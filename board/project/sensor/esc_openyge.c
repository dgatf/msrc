#include "esc_openyge.h"

#include <math.h>
#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

#define TIMEOUT_US 2000
#define FRAME_START 0xA5
#define CRC_POLYNOMIAL 0x1021  // CRC-16-CCITT
#define OPENYGE_INTERVAL_MS 50

typedef struct openyge_telemetry_request_t {
    uint8_t sync;     // Source device 0xA5
    uint8_t version;  // Version 0x03
    uint8_t type;     // Type 0x03
    uint8_t len;      // Length of the frame
    uint8_t seq;
    uint8_t device;
    uint16_t index;  // Index for telemetry data (0x0000 for first request)
    uint16_t data;   // Data for telemetry request (0x0000 for first request)
    uint16_t crc;    // CRC-16-CCITT
} __attribute__((packed)) openyge_telemetry_request_t;

typedef struct openyge_telemetry_payload_t {
    uint8_t reserved1;     // Reserved
    uint8_t temp_fet;      // FET temperature (byte 7)
    uint16_t voltage;      // Voltage (bytes 8-9, little endian)
    uint16_t current;      // Current (bytes 10-11, little endian)
    uint16_t consumption;  // Consumption (bytes 12-13, little endian)
    uint16_t erpm;         // eRPM (bytes 14-15, little endian)
    uint8_t pwm_percent;   // PWM percentage (byte 16)
    uint8_t throttle;      // Throttle percentage (byte 17)
    uint16_t voltage_bec;  // BEC voltage (bytes 18-19, little endian)
    uint16_t current_bec;  // BEC current (bytes 20-21, little endian)
    uint8_t temp_bec;      // BEC/Cap temperature (byte 22)
    uint8_t status1;
    uint8_t cap_temp;
    uint8_t aux_temp;
    uint8_t status2;
    uint8_t reserved2;
    uint16_t pidx;
    uint16_t pdata;
} __attribute__((packed)) openyge_telemetry_payload_t;

typedef struct openyge_telemetry_answer_t {
    uint8_t sync;     // Source device 0xA5
    uint8_t version;  // Version 0x03
    uint8_t type;     // Type 0x02
    uint8_t len;      // Length of the frame (8+26=34)
    uint8_t seq;
    uint8_t device;
    openyge_telemetry_payload_t payload;  // Payload data
    uint16_t crc;                         // CRC-16-CCITT
} __attribute__((packed)) openyge_telemetry_answer_t;

static void process(esc_openyge_parameters_t *parameter);
static void send_packet(void);
static uint16_t calculate_crc16(uint8_t *data, uint8_t length);

void esc_openyge_task(void *parameters) {
    esc_openyge_parameters_t parameter = *(esc_openyge_parameters_t *)parameters;

    // Initialize all output variables
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temperature_fet = 0;
    *parameter.temperature_bec = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.voltage_bec = 0;
    *parameter.current_bec = 0;
    *parameter.throttle = 0;
    *parameter.pwm_percent = 0;
    *parameter.cell_count = 1;

    xTaskNotifyGive(context.receiver_task_handle);

#ifdef SIM_SENSORS
    *parameter.rpm = 12345.67;
    *parameter.consumption = 123.4;
    *parameter.voltage = 22.2;
    *parameter.current = 15.5;
    *parameter.temperature_fet = 45.6;
    *parameter.temperature_bec = 38.2;
    *parameter.cell_voltage = 3.7;
    *parameter.voltage_bec = 5.1;
    *parameter.current_bec = 0.8;
    *parameter.throttle = 75;
    *parameter.pwm_percent = 80;
#endif

    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);

    debug("\nOpenYGE ESC init");

    while (1) {
        send_packet();
        if (ulTaskNotifyTakeIndexed(1, pdTRUE, OPENYGE_INTERVAL_MS / portTICK_PERIOD_MS)) {
            process(&parameter);
            vTaskDelay(OPENYGE_INTERVAL_MS / portTICK_PERIOD_MS);
        }
    }
}

static void process(esc_openyge_parameters_t *parameter) {
    uint8_t length = uart1_available();

    if (length != sizeof(openyge_telemetry_answer_t)) return;

    openyge_telemetry_answer_t frame;
    uart1_read_bytes((uint8_t *)&frame, length);

    debug("\nOpenYGE (%u) < ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer((uint8_t *)&frame, length, "0x%02X ");

    uint16_t crc = calculate_crc16((uint8_t *)&frame, sizeof(frame) - 2);

    if (frame.sync == FRAME_START && frame.type == 0x02 /*&& frame.device == 0x81*/ && frame.crc == crc) {
        int temp_fet_raw = (int)frame.payload.temp_fet - 40;
        float temperature_fet = (temp_fet_raw < -40 || temp_fet_raw > 215) ? 0 : temp_fet_raw;
        uint16_t voltage_raw = frame.payload.voltage;
        float voltage = voltage_raw * 0.01f;
        uint16_t current_raw = frame.payload.current;
        float current = current_raw * 0.01f;
        uint16_t consumption = frame.payload.consumption;
        uint16_t erpm_raw = frame.payload.erpm;
        float erpm = erpm_raw * 10.0f;
        float rpm = erpm * parameter->rpm_multiplier;
        float pwm_percent = frame.payload.pwm_percent;
        float throttle = frame.payload.throttle;
        uint16_t bec_voltage_raw = frame.payload.voltage_bec;
        float voltage_bec = bec_voltage_raw / 1000.0f;
        uint16_t bec_current_raw = frame.payload.current_bec;
        float current_bec = bec_current_raw / 1000.0f;
        int temp_bec_raw = (int)frame.payload.temp_bec - 40;
        float temperature_bec = (temp_bec_raw < -40 || temp_bec_raw > 215) ? 0 : temp_bec_raw;

        // Update outputs with averaging
        *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
        *parameter->consumption = consumption;
        *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
        *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
        *parameter->temperature_fet =
            get_average(parameter->alpha_temperature, *parameter->temperature_fet, temperature_fet);
        *parameter->temperature_bec =
            get_average(parameter->alpha_temperature, *parameter->temperature_bec, temperature_bec);
        *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
        *parameter->voltage_bec = get_average(parameter->alpha_voltage, *parameter->voltage_bec, voltage_bec);
        *parameter->current_bec = get_average(parameter->alpha_current, *parameter->current_bec, current_bec);
        *parameter->throttle = throttle;
        *parameter->pwm_percent = pwm_percent;

        debug(
            "\nOpenYGE (%u) < Dev:0x%02X RPM: %.0f Volt: %.2fV Curr: %.2fA TempFET: %.1f°C "
            "TempBEC: %.1f°C Cons: %.0fmAh CellV: %.2fV BECVolt: %.2fV BECCurr: %.3fA Thr: %.0f%% PWM: %.0f%%",
            uxTaskGetStackHighWaterMark(NULL), frame.device, *parameter->rpm, *parameter->voltage, *parameter->current,
            *parameter->temperature_fet, *parameter->temperature_bec, *parameter->consumption, *parameter->cell_voltage,
            *parameter->voltage_bec, *parameter->current_bec, *parameter->throttle, *parameter->pwm_percent);

        return;  // Successfully processed frame
    } else {
        debug("\nOpenYGE invalid frame");
    }
}

static void send_packet(void) {
    static uint8_t seq = 0;
    openyge_telemetry_request_t request = {
        .sync = 0xA5,
        .version = 0x03,
        .type = 0x03,
        .len = sizeof(openyge_telemetry_request_t),
        .seq = seq++,
        .device = 0x81,  // esc id
        .index = 0,
        .data = 0
    };
    request.crc = calculate_crc16((uint8_t *)&request, sizeof(request) - 2);
    uart1_write_bytes((uint8_t *)&request, sizeof(request));
}

static uint16_t calculate_crc16(uint8_t *data, uint8_t length) {
    uint16_t crc = 0x0000;

    for (uint8_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
