#include "esc_flycolor.h"

#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "string.h"
#include "uart.h"

#define FLYCOLOR_TIMEOUT_US 5000
#define FLYCOLOR_PACKET_LENGHT 14
#define FLYCOLOR_HEADER 0x53
#define FLYCOLOR_REQUEST 0xAA
#define FLYCOLOR_INTERVAL_MS 200
#define FLYCOLOR_INIT_DELAY_MS 15000

typedef struct esc_flycolor_t {
    uint8_t header;     // Header 0x53
    uint16_t temp;      // Temp, C,  x100
    uint16_t current;   // Current, A, x10
    uint16_t unk;       // Unknown
    uint16_t rpm;       // rpm, /10
    uint16_t throttle;  // throttle, us
    uint8_t pwm;        // pwm, %
    uint16_t voltage;   // voltage, V, x1000
} __attribute__((packed)) esc_flycolor_t;

static void process(esc_flycolor_parameters_t *parameter);
static int64_t alarm_packet(alarm_id_t id, void *parameter);
static void do_handshake(uint8_t *buffer, uint length);

void esc_flycolor_task(void *parameters) {
    esc_flycolor_parameters_t parameter = *(esc_flycolor_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temperature = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.cell_count = 1;
    parameter.request_telemetry = false;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.temperature = 12.34;
    *parameter.voltage = 12.34;
    *parameter.current = 12.34;
    *parameter.consumption = 12.34;
    *parameter.rpm = 12345.67;
    *parameter.cell_voltage = 3.75;
    *parameter.voltage_bec = 2.34;
    ;
#endif

    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    uart1_begin(19200, UART1_TX_GPIO, UART_ESC_RX, FLYCOLOR_TIMEOUT_US, 8, 1, UART_PARITY_ODD, false, true);

    uint8_t handshake_answer[256];
    uint length;
    vTaskDelay(FLYCOLOR_INIT_DELAY_MS / portTICK_PERIOD_MS);

    uint8_t handshake0[] = {0xA5};
    do_handshake(handshake0, sizeof(handshake0));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    length = uart1_available();
    if (length) {
        uart1_read_bytes(handshake_answer, length);
        debug("\nFlycolor (%u). Handshake < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(handshake_answer, length, "0x%X ");
    }

    uint8_t handshake1[] = {0x0, 0x2F, 0xF5};
    do_handshake(handshake1, sizeof(handshake1));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    length = uart1_available();
    if (length) {
        uart1_read_bytes(handshake_answer, length);
        debug("\nFlycolor (%u). Handshake < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(handshake_answer, length, "0x%X ");
    }
    uint8_t handshake2[] = {0x0, 0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x6A, 0xFF};
    do_handshake(handshake2, sizeof(handshake2));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    length = uart1_available();
    if (length) {
        uart1_read_bytes(handshake_answer, length);
        debug("\nFlycolor (%u). Handshake < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(handshake_answer, length, "0x%X ");
    }
    uint8_t handshake3[] = {0x46, 0xA6, 0x56, 0xA8, 0x8A, 0xF5, 0x49, 0x22, 0x8B};
    do_handshake(handshake3, sizeof(handshake3));
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    length = uart1_available();
    if (length) {
        uart1_read_bytes(handshake_answer, length);
        debug("\nFlycolor (%u). Handshake < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(handshake_answer, length, "0x%X ");
    }
    add_alarm_in_us(1000, alarm_packet, &parameter, true);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
        if (parameter.request_telemetry) {
            parameter.request_telemetry = false;
            uart1_write(FLYCOLOR_REQUEST);
            debug("\nFlycolor (%u) > 0x%X", uxTaskGetStackHighWaterMark(NULL), FLYCOLOR_REQUEST);
        }
    }
}

static void do_handshake(uint8_t *buffer, uint length) {
    // uart1_write_bytes(buffer, length);
    for (uint i = 0; i < length; i++) {
        uart1_write(*(buffer + i));
        vTaskDelay(3 / portTICK_PERIOD_MS);

    }
    debug("\nFlycolor (%u). Handshake > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer(buffer, length, "0x%X ");
}

static void process(esc_flycolor_parameters_t *parameter) {
    uint8_t length = uart1_available();
    uint8_t buffer[64];
    uart1_read_bytes(buffer, length);
    if (length) {
        debug("\nFlycolor (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, length, "0x%X ");
    }
    if (length == FLYCOLOR_PACKET_LENGHT) {
        esc_flycolor_t packet;
        static uint32_t timestamp = 0;
        memcpy(&packet, buffer, sizeof(packet));
        if (packet.header != FLYCOLOR_HEADER) return;
        packet.temp = swap_16(packet.temp);
        packet.current = swap_16(packet.current);
        packet.unk = swap_16(packet.unk);
        packet.rpm = swap_16(packet.rpm);
        packet.voltage = swap_16(packet.voltage);
        *parameter->temperature =
            get_average(parameter->alpha_temperature, *parameter->temperature, packet.temp / 100.0);
        *parameter->current = get_average(parameter->alpha_current, *parameter->current, packet.current / 100.0);
        *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, packet.rpm * 10.0);
        *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, packet.voltage / 1000.0);
        *parameter->consumption += get_consumption(*parameter->current, 0, &timestamp);
        *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
        debug("\nFlycolor (%u) < Rpm: %.0f Volt: %0.3f Curr: %.2f Temp: %.2f Unk: %u Cons: %.0f CellV: %.2f",
              uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, *parameter->current,
              *parameter->temperature, swap_16(packet.unk), *parameter->consumption, *parameter->cell_voltage);
    }
}

static int64_t alarm_packet(alarm_id_t id, void *parameter) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    esc_flycolor_parameters_t *parameters = (esc_flycolor_parameters_t *)parameter;
    parameters->request_telemetry = true;
    vTaskNotifyGiveIndexedFromISR(context.uart1_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return FLYCOLOR_INTERVAL_MS * 1000;
}