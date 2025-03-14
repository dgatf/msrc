#include "fuel_meter.h"

#include <stdio.h>

#include "capture_edge.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#define INSTANT_INTERVAL_MS 100
#define CLOCK_DIV 5

static volatile uint pwm_cycles_instant = 0;
static volatile uint pwm_cycles_total = 0;

static void read(fuel_meter_parameters_t *parameter);
static void capture_pin_0_handler(uint counter, edge_type_t edge);

void fuel_meter_task(void *parameters) {
    fuel_meter_parameters_t parameter = *(fuel_meter_parameters_t *)parameters;
    *parameter.consumption_instant = 0;  // ml/min
    *parameter.consumption_total = 0;    // ml
    xTaskNotifyGive(context.receiver_task_handle);

    gpio_pull_up(FUELMETER_CAPTURE_GPIO);

    capture_edge_init(pio0, FUELMETER_CAPTURE_GPIO, CLOCK_DIV, PIO0_IRQ_0);
    capture_edge_set_handler(0, capture_pin_0_handler);

    while (1) {
        read(&parameter);
        debug("\nFuel sensor (%u) < ml/min %.3f ml %.3f ml/pulse %.3f pulses %u", uxTaskGetStackHighWaterMark(NULL), *parameter.consumption_instant,
              *parameter.consumption_total, parameter.ml_per_pulse, pwm_cycles_total);
        vTaskDelay(INSTANT_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void read(fuel_meter_parameters_t *parameter) {
    *parameter->consumption_total = pwm_cycles_total * parameter->ml_per_pulse;  // ml
    *parameter->consumption_instant =
        pwm_cycles_instant * parameter->ml_per_pulse * 1000 / INSTANT_INTERVAL_MS * 60;  // ml/min
    pwm_cycles_instant = 0;
#ifdef SIM_SENSORS
    *parameter->consumption_instant = 12.23;
    *parameter->consumption_total = 1223;
#endif
}

static void capture_pin_0_handler(uint counter, edge_type_t edge) {
    if (edge == EDGE_RISE) {
        pwm_cycles_total++;
        pwm_cycles_instant++;
    }
}

