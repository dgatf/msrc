#include "esc_hw3.h"

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "uart.h"

#define TIMEOUT_US 1000
#define PACKET_LENGHT 10
#define NO_SIGNAL_TIMEOUT_MS 500

static void process(esc_hw3_parameters_t *parameter);
static int64_t timeout_callback(alarm_id_t id, void *parameters);

void esc_hw3_task(void *parameters) {
    debug("\nHW3 init");
    esc_hw3_parameters_t parameter = *(esc_hw3_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    *parameter.rpm = 0;
#ifdef SIM_SENSORS
    *parameter.rpm = 12345.67;
#endif
    uart1_begin(19200, UART1_TX_GPIO, UART_ESC_RX, TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(esc_hw3_parameters_t *parameter) {
    static alarm_id_t timeout_alarm_id = 0;
    if (uart1_available() == PACKET_LENGHT) {
        if (timeout_alarm_id) cancel_alarm(timeout_alarm_id);
        uint8_t data[PACKET_LENGHT];
        uart1_read_bytes(data, PACKET_LENGHT);
        if (data[0] == 0x9B && data[4] == 0 && data[6] == 0) {
            uint16_t rpmCycle = (uint16_t)data[8] << 8 | data[9];
            if (rpmCycle <= 0) rpmCycle = 1;
            float rpm = 60000000.0 / rpmCycle * parameter->multiplier;
            *parameter->rpm = get_average(parameter->alpha, *parameter->rpm, rpm);
            uint32_t packet = (uint32_t)data[1] << 16 | (uint16_t)data[2] << 8 | data[3];
            debug("\nEsc HW3 (%u) < Packet: %i Rpm: %.0f", uxTaskGetStackHighWaterMark(NULL), packet, *parameter->rpm);
        }
        timeout_alarm_id = add_alarm_in_ms(NO_SIGNAL_TIMEOUT_MS, timeout_callback, parameter->rpm, false);
    }
}

static int64_t timeout_callback(alarm_id_t id, void *parameters) {
    float *parameter = (float *)parameters;
    *parameter = 0;
    debug("\nEsc HW3 signal timeout. Rpm: 0");
    return 0;
}