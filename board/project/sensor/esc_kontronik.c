#include "esc_kontronik.h"

#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

#define TIMEOUT_US 1000
#define PACKET_LENGHT 35

static void process(esc_kontronik_parameters_t *parameter);

void esc_kontronik_task(void *parameters) {
    esc_kontronik_parameters_t parameter = *(esc_kontronik_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.voltage_bec = 0;
    *parameter.current_bec = 0;
    *parameter.temperature_fet = 0;
    *parameter.temperature_bec = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.cell_count = 1;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.rpm = 12345.67;
    *parameter.consumption = 123.4;
    *parameter.voltage = 12.34;
    *parameter.current = 12.34;
    *parameter.temperature_fet = 12.34;
    *parameter.temperature_bec = 12.34;
    *parameter.cell_voltage = 3.75;
#endif

    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, TIMEOUT_US, 8, 1, UART_PARITY_EVEN, false, false);
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(esc_kontronik_parameters_t *parameter) {
    static uint32_t timestamp = 0;
    if (uart1_available() == PACKET_LENGHT) {
        uint8_t data[PACKET_LENGHT];
        uart1_read_bytes(data, PACKET_LENGHT);
        if (data[0] == 0x4B && data[1] == 0x4F && data[2] == 0x44 && data[3] == 0x4C) {
            float rpm = (uint32_t)data[7] << 24 | (uint32_t)data[6] << 16 | (uint16_t)data[5] << 8 | data[4];
            rpm *= parameter->rpm_multiplier;
            float voltage = ((uint16_t)data[9] << 8 | data[8]) / 100.0;
            float current = ((uint16_t)data[11] << 8 | data[10]) / 10.0;
            float consumption = ((uint16_t)data[17] << 8 | data[16]);
            float current_bec = ((uint16_t)data[19] << 8 | data[18]) / 1000.0;
            float voltage_bec = ((uint16_t)data[21] << 8 | data[20]) / 1000.0;
            float temperature_fet = data[26];
            float temperature_bec = data[27];
            *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
            *parameter->consumption = consumption;
            *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
            *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
            *parameter->voltage_bec = get_average(parameter->alpha_voltage, *parameter->voltage_bec, voltage_bec);
            *parameter->current_bec = get_average(parameter->alpha_current, *parameter->current_bec, current_bec);
            *parameter->temperature_fet =
                get_average(parameter->alpha_temperature, *parameter->temperature_fet, temperature_fet);
            *parameter->temperature_bec =
                get_average(parameter->alpha_temperature, *parameter->temperature_bec, temperature_bec);
            *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
            debug(
                "\nKontronic (%u) < Rpm: %.0f Volt: %0.2f Curr: %.2f V Bec: %0.2f C Bec: %.2f TempFet: %.0f TempBec: "
                "%.0f Cons: %.0f CellV: %.2f",
                uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, *parameter->current,
                *parameter->voltage_bec, *parameter->current_bec, *parameter->temperature_fet,
                *parameter->temperature_bec, *parameter->consumption, *parameter->cell_voltage);
        }
    }
}