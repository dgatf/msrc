#ifndef ESC_HW4_H
#define ESC_HW4_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "uart.h"
#include "cell_count.h"
#include "auto_offset.h"
#include "common.h"
#include "config.h"

#include "uart_pio.h"

#define ESC_HW4_TIMEOUT_US 2000
#define ESC_HW4_PACKET_LENGHT 19
#define ESC_HW4_SIGNATURE_LENGHT 13
#define ESC_HW4_CURRENT_OFFSET_DELAY 15000

#define ESC_HW4_NTC_BETA 3950.0
#define ESC_HW4_NTC_R1 10000.0
#define ESC_HW4_NTC_R_REF 47000.0
#define ESC_HW4_DIFFAMP_SHUNT (0.25 / 1000)
#define ESC_HW4_V_REF 3.3
#define ESC_HW4_ADC_RES 4096.0

/* ESCHW4_DIVISOR and ESCHW4_AMPGAIN values

Divisor: Cells range

3-6S (LV): divisor = 11
3-8S (LV v2): divisor = 15.4
5-12s (HV): divisor = 21

Gain: Amperage

60A: gain = 6
80A: gain = 7.8
100A: gain = 9(1)
120A: gain = 10
130A: gain = 11.3(1)
150A: gain = 12.9(1)
160A: gain = 13.7(1)
200A: gain = 16.9

(1) Extrapolated from confirmed models

Current multiplier = 1 / (ampgain* 0.25/1000) = 4000 / ampgain

*/

typedef struct esc_hw4_parameters_t
{
    float rpm_multiplier;
    bool pwm_out;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float divisor, current_multiplier, current_thresold, current_max;
    bool current_is_manual_offset;
    float current_offset;    
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *cell_voltage, *consumption;
    uint8_t *cell_count;
} esc_hw4_parameters_t;


extern TaskHandle_t pwm_out_task_handle, receiver_task_handle;
extern QueueHandle_t tasks_queue_handle;
extern uint8_t debug;

void esc_hw4_task(void *parameters);

#endif