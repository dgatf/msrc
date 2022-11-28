#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "capture_edge.h"
#include "common.h"
#include "esc_hw3.h"
#include "config.h"

#define ESC_PWM_SIGNAL_TIMEOUT_MS 1000
#define ESC_PWM_INTERVAL_MS 100
#define ESC_PWM_CLOCK_DIV 1

typedef esc_hw3_parameters_t esc_pwm_parameters_t;

void esc_pwm_task(void *parameters);

#endif