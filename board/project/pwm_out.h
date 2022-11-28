#ifndef PWM_OUT_H
#define PWM_OUT_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "common.h"
#include "config.h"

extern TaskHandle_t receiver_task_handle;
extern uint8_t debug;

void pwm_out_task(void *parameters);

#endif
