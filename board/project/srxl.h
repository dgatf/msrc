#ifndef SRXL_H
#define SRXL_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

#include "i2c_multi.h"
#include "common.h"
#include "uart.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "esc_vbar.h"
#include "voltage.h"
#include "current.h"
#include "ntc.h"
#include "airspeed.h"
#include "esc_kontronik.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "nmea.h"
#include "esc_pwm.h"
#include "bmp280.h"
#include "ms5611.h"
#include "bmp180.h"
#include "pwm_out.h"
#include "xbus.h"

#define SRXL_HEADER 0xA5
#define SRXL_FRAMELEN 18
#define SRXL_TIMEOUT_US 1000

extern QueueHandle_t tasks_queue_handle;
extern TaskHandle_t pwm_out_task_handle, led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;

extern xbus_sensor_t *sensor;
extern xbus_sensor_formatted_t *sensor_formatted;

void srxl_task(void *parameters);

#endif