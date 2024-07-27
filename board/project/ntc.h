#ifndef NTC_H
#define NTC_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "common.h"
#include "config.h"

// Thermistors (NTC 100k, R1 10k)
#define NTC_R_REF 100000UL
#define NTC_R1 10000
#define NTC_BETA 4190
//#define NTC_A1 3.35E-03
//#define NTC_B1 2.46E-04
//#define NTC_C1 3.41E-06
//#define NTC_D1 1.03E-07

typedef struct ntc_parameters_t
{
    uint8_t adc_num;
    uint8_t rate;
    float alpha;
    float *ntc;

} ntc_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern uint8_t debug;

void ntc_task(void *parameters);

#endif