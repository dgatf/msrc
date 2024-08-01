#ifndef ESC_PWM_H
#define ESC_PWM_H

#include "esc_hw3.h"

typedef esc_hw3_parameters_t esc_pwm_parameters_t;

void esc_pwm_task(void *parameters);

#endif