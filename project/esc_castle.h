
#ifndef ESC_CASTLE_H
#define ESC_CASTLE_H

/* castleTelemetry:
    index   element                          scaler
    0       calib 1 (1000us)                 0
    1       Volt (V)                         20
    2       rippleVolt (V)                   4
    3       Curr (A)                         50
    4       Thr (0.1-0.2 ms esc pulse)       1
    5       Output power (%)                 0.2502
    6       rpm                              20416.7
    7       becVolt (V)                      4
    8       becCurr (A)                      4
    9       temp (C) or calib 2 (500us)      30
    10      temp ntc (C) or calib 2 (500us)  63.8125
*/

#define RX_MAX_CYCLES 2 // minimum is 2

#define CASTLE_R0 10000.0F
#define CASTLE_R2 10200.0F
#define CASTLE_B 3455.0F

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "castle_link.h"
#include "cell_count.h"
#include "common.h"
#include "config.h"

#define ESC_CASTLE_NO_SIGNAL_TIMEOUT 1000
#define ESC_CASTLE_TELEMETRY_TIMEOUT 10
#define ESC_CASTLE_CLOCK_DIV 50

typedef struct esc_castle_parameters_t
{
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *voltage, *ripple_voltage, *current, *thr, *output, *rpm, *consumption, *voltage_bec, *current_bec, *temperature, *cell_voltage;
    uint8_t *cell_count;
} esc_castle_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;

void esc_castle_task(void *parameters);

#endif
