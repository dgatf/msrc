#include "esc_castle.h"

#include <semphr.h>
#include <stdio.h>

#include "cell_count.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

static volatile esc_castle_parameters_t parameter;

static void castle_link_handler(castle_link_telemetry_t packet);

void esc_castle_task(void *parameters) {
    parameter = *(esc_castle_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.ripple_voltage = 0;
    *parameter.current = 0;
    *parameter.thr = 0;
    *parameter.output = 0;
    *parameter.rpm = 0;
    *parameter.consumption = 0;
    *parameter.voltage_bec = 0;
    *parameter.current_bec = 0;
    *parameter.temperature = 0;
    *parameter.cell_voltage = 0;
    *parameter.cell_count = 1;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.voltage = 12.34;
    *parameter.ripple_voltage = 1.23;
    *parameter.current = 23.45;
    *parameter.thr = 50;
    *parameter.output = 80;
    *parameter.rpm = 12345.67;
    *parameter.consumption = 345.67;
    *parameter.voltage_bec = 4.56;
    *parameter.current_bec = 5.67;
    *parameter.temperature = 29.9;
    *parameter.cell_voltage = 3.75;
#endif

    castle_link_init(pio0, CASTLE_PWM_GPIO, PIO0_IRQ_0);
    castle_link_set_handler(castle_link_handler);
    debug("\nCastle init");
    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    vTaskSuspend(NULL);
    vTaskDelete(NULL);
}

static void castle_link_handler(castle_link_telemetry_t packet) {
    *parameter.voltage = packet.voltage;
    *parameter.ripple_voltage = packet.ripple_voltage;
    *parameter.current = packet.current;
    *parameter.thr = packet.thr;
    *parameter.output = packet.output;
    *parameter.rpm = packet.rpm;
    *parameter.voltage_bec = packet.voltage_bec;
    *parameter.current_bec = packet.current_bec;
    *parameter.temperature = packet.temperature;
    debug(
        "\nCastle (%u) < Volt(V): %.2f Ripple volt(V): %.2f Curr(A): %.2f Thr: %.0f Out: %.0f Rpm: %.0f Bec volt(V): "
        "%.2f Bec curr(A): %.2f Temp(C): %.0f %s",
        uxTaskGetStackHighWaterMark(NULL), packet.voltage, packet.ripple_voltage, packet.current, packet.thr,
        packet.output, packet.rpm, packet.voltage_bec, packet.current_bec, packet.temperature,
        packet.is_temp_ntc ? " NTC" : " Linear");
}
