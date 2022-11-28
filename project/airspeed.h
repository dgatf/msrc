#ifndef AIRSPEED_H
#define AIRSPEED_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "common.h"
#include "config.h"

#define AIR_DENS 1.204  // 20ºC, 1atm
#define KNOT_TO_MS 1.94384
// Transfer function: P(Pa) = 1000 * (Vo/(SLOPE*VCC) - voltageOffset)
// MPXV7002
#define TRANSFER_SLOPE 0.2
#define TRANSFER_VCC 5

typedef struct airspeed_parameters_t
{
    uint8_t adc_num;
    float alpha;
    float *airspeed;

} airspeed_parameters_t;

extern TaskHandle_t receiver_task_handle;
extern uint8_t debug;

void airspeed_task(void *parameters);

#endif