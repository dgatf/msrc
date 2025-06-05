#include "esc_pwm.h"

#include <stdio.h>

#include "capture_edge.h"
#include "esc_hw3.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#define SIGNAL_TIMEOUT_MS 1000
#define INTERVAL_MS 100
#define CLOCK_DIV 1

static volatile bool is_timedout = false;
static volatile uint pwm_cycles = 0;

static void read(esc_pwm_parameters_t *parameter);
static void capture_pin_0_handler(uint counter, edge_type_t edge);
static int64_t timeout_callback(alarm_id_t id, void *user_data);

void esc_pwm_task(void *parameters) {
    esc_pwm_parameters_t parameter = *(esc_pwm_parameters_t *)parameters;
    *parameter.rpm = 0;
    xTaskNotifyGive(context.receiver_task_handle);

    gpio_pull_up(PWM_CAPTURE_GPIO);

    capture_edge_init(pio0, PWM_CAPTURE_GPIO, 1, CLOCK_DIV, PIO0_IRQ_0);
    capture_edge_set_handler(0, capture_pin_0_handler);

    while (1) {
        read(&parameter);
        debug("\nEsc PWM (%u) < Rpm: %.0f", uxTaskGetStackHighWaterMark(NULL), *parameter.rpm);
        vTaskDelay(INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void read(esc_pwm_parameters_t *parameter) {
    if (is_timedout) {
        *parameter->rpm = 0;
        return;
    }
    if (pwm_cycles > 1) {
        float pwm_duration = (float)pwm_cycles / clock_get_hz(clk_sys) * CLOCK_DIV * COUNTER_CYCLES;  // seconds
        float rpm = 60 / pwm_duration * parameter->multiplier;
        *parameter->rpm = rpm;  // get_average(parameter->alpha / 100.0F, *parameter->rpm, rpm);
    }
#ifdef SIM_SENSORS
    *parameter->rpm = 12345.67;
#endif
}

static void capture_pin_0_handler(uint counter, edge_type_t edge) {
    static uint counter_edge_rise = 0, counter_previous = 0;
    static alarm_id_t timeout_alarm_id = 0;
    if (timeout_alarm_id) cancel_alarm(timeout_alarm_id);
    is_timedout = false;

    if (edge == EDGE_RISE) {
        pwm_cycles = counter - counter_edge_rise;
        counter_edge_rise = counter;
        // printf("\nPwm cycles: %u", pwm_cycles);
    }
    timeout_alarm_id = add_alarm_in_ms(SIGNAL_TIMEOUT_MS, timeout_callback, NULL, false);
}

static int64_t timeout_callback(alarm_id_t id, void *parameters) {
    is_timedout = true;
    debug("\nEsc PWM signal timeout. Rpm: 0");
    return 0;
}
