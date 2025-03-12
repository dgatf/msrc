#include "esc_apd_hv.h"

#include <math.h>
#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

#define ESC_KISS_PACKET_LENGHT 10
#define ESC_APD_HV_TIMEOUT_US 1000
#define ESC_APD_HV_PACKET_LENGHT 22

static void process(esc_apd_hv_parameters_t *parameter);
static float get_temperature(uint16_t raw);
static uint16_t get_crc16(uint8_t *buffer);

void esc_apd_hv_task(void *parameters) {
    esc_apd_hv_parameters_t parameter = *(esc_apd_hv_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temperature = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.cell_count = 1;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.temperature = 12.34;
    *parameter.voltage = 12.34;
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

    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, ESC_APD_HV_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(esc_apd_hv_parameters_t *parameter) {
    static uint32_t timestamp = 0;
    if (uart1_available() == ESC_APD_HV_PACKET_LENGHT) {
        uint8_t data[ESC_APD_HV_PACKET_LENGHT];
        uart1_read_bytes(data, ESC_APD_HV_PACKET_LENGHT);
        if (get_crc16(data) == (((uint16_t)data[19] << 8) | data[18])) {
            float voltage = ((uint16_t)data[1] << 8 | data[0]) / 100.0;
            float temp = get_temperature((uint16_t)data[3] << 8 | data[2]);
            float current = ((uint16_t)data[5] << 8 | data[4]) / 12.5;
            float rpm = (uint32_t)data[11] << 24 | (uint32_t)data[10] << 16 | (uint16_t)data[9] << 8 | data[8];
            *parameter->temperature = get_average(parameter->alpha_temperature, *parameter->temperature, temp);
            *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
            *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
            *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
            *parameter->consumption += get_consumption(*parameter->current, 0, &timestamp);
            *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
            debug("\nApd HV (%u) < Rpm: %.0f Volt: %0.2f Curr: %.2f Temp: %.0f Cons: %.0f CellV: %.2f",
                  uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, *parameter->current,
                  *parameter->temperature, *parameter->consumption, *parameter->cell_voltage);
        }
    }
}

static float get_temperature(uint16_t raw) {
    uint16_t SERIESRESISTOR = 10000;
    uint16_t NOMINAL_RESISTANCE = 10000;
    uint8_t NOMINAL_TEMPERATURE = 25;
    uint16_t BCOEFFICIENT = 3455;

    // convert value to resistance
    float Rntc = (4096 / (float)raw) - 1;
    Rntc = SERIESRESISTOR / Rntc;

    // Get the temperature
    float temperature = Rntc / (float)NOMINAL_RESISTANCE;  // (R/Ro)
    temperature = (float)log(temperature);                 // ln(R/Ro)
    temperature /= BCOEFFICIENT;                           // 1/B * ln(R/Ro)

    temperature += (float)1.0 / ((float)NOMINAL_TEMPERATURE + (float)273.15);  // + (1/To)
    temperature = (float)1.0 / temperature;                                    // Invert
    temperature -= (float)273.15;
    return temperature;
}

static uint16_t get_crc16(uint8_t *buffer) {
    uint16_t fCCRC16;
    uint16_t c0 = 0;
    uint16_t c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (uint8_t i = 0; i < 18; i++)  // Check only first 18 bytes, skip crc bytes
    {
        c0 = (uint16_t)(c0 + (buffer[i])) % 255;
        c1 = (uint16_t)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = (c1 << 8) | c0;
    return fCCRC16;
}