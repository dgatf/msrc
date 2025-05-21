#include "esc_omp_m4.h"

#include <stdio.h>

#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"

#define OMP_M4_TIMEOUT_US 1000
#define OMP_M4_PACKET_LENGHT 32
#define OMP_M4_PACKET_HEADER 0xDD

typedef struct esc_omp_m4_packet_t {
    uint8_t header;
    uint8_t version;
    uint8_t length;
    uint16_t voltage;
    uint16_t current;
    uint8_t throttle;
    uint16_t rpm;
    uint8_t temp_esc;
    uint8_t temp_motor;
    uint8_t pwm;
    uint16_t status;
    uint16_t consumption;
    uint8_t unused[31 - 17 + 1];
} __attribute((packed)) esc_omp_m4_packet_t;

static void process(esc_omp_m4_parameters_t *parameter);

void esc_omp_m4_task(void *parameters) {
    esc_omp_m4_parameters_t parameter = *(esc_omp_m4_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temp_esc = 0;
    *parameter.temp_motor = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.cell_count = 1;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.temperature_esc = 12.34;
    *parameter.temperature_motor = 23.45;
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

    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, OMP_M4_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(esc_omp_m4_parameters_t *parameter) {
    static uint32_t timestamp = 0;
    uint8_t lenght = uart1_available();
    if (lenght == OMP_M4_PACKET_LENGHT) {
        esc_omp_m4_packet_t packet;
        uart1_read_bytes((uint8_t *)&packet, OMP_M4_PACKET_LENGHT);
        if (packet.header != OMP_M4_PACKET_HEADER) return;
        float temp_esc = packet.temp_esc;
        float temp_motor = packet.temp_motor;
        float voltage = swap_16(packet.voltage) / 10.0;
        float current = swap_16(packet.current) / 10.0;
        float consumption = swap_16(packet.consumption);
        float rpm = swap_16(packet.rpm) * 10.0;
        rpm *= parameter->rpm_multiplier;
        *parameter->temp_esc = get_average(parameter->alpha_temperature, *parameter->temp_esc, temp_esc);
        *parameter->temp_motor = get_average(parameter->alpha_temperature, *parameter->temp_motor, temp_motor);
        *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
        *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
        *parameter->consumption = get_average(parameter->alpha_voltage, *parameter->consumption, consumption);
        *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
        *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
        debug("\nOMP M4 (%u) < Rpm: %.0f Volt: %.1f Curr: %.1f Temp esc: %.0f Temp motor: %.0f Cons: %.0f CellV: %.2f",
              uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, *parameter->current,
              *parameter->temp_esc, *parameter->temp_motor, *parameter->consumption, *parameter->cell_voltage);
    }
}
