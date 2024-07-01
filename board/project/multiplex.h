#ifndef MULTIPLEX_H
#define MULTIPLEX_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"

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

/* Flysky MULTIPLEX FHSS Data Id */
#define FHSS_VOLTAGE 1
#define FHSS_CURRENT 2
#define FHSS_VARIO 3
#define FHSS_SPEED 4
#define FHSS_RPM 5
#define FHSS_TEMP 6
#define FHSS_COURSE 7
#define FHSS_ALTITUDE 8
#define FHSS_LEVEL 9
#define FHSS_RSSI 10
#define FHSS_CONSUMPTION 11
#define FHSS_FLUID 12
#define FHSS_DISTANCE 13

#define MULTIPLEX_RECEIVED_NONE 0
#define MULTIPLEX_RECEIVED_POLL 1

#define MULTIPLEX_COMMAND_DISCOVER 0x8
#define MULTIPLEX_COMMAND_TYPE 0x9
#define MULTIPLEX_COMMAND_MEASURE 0xA

#define MULTIPLEX_TIMEOUT_US 1000
#define MULTIPLEX_PACKET_LENGHT 1

typedef struct sensor_multiplex_t
{
    uint8_t data_id;
    float *value;
} sensor_multiplex_t;

extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;
extern TaskHandle_t pwm_out_task_handle, led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;
extern uint8_t debug;

void multiplex_task(void *parameters);

#endif