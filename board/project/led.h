#ifndef LED_H
#define LED_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "config.h"
#include "ws2812.h"

extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;

extern uint8_t debug;

void led_task();

#endif