#ifndef USB_H
#define USB_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <queue.h>
#include "config.h"
#include "constants.h"

#define USB_BUFFER_LENGTH 256
#define USB_INTERVAL_MS 1000

extern TaskHandle_t led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;
extern uint8_t debug;

void usb_task();

#endif
