#ifndef JETIEX_H
#define JETIEX_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "common.h"
#include "uart.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
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

#define JETIEX_TYPE_INT6 0
#define JETIEX_TYPE_INT14 1
#define JETIEX_TYPE_INT22 4
#define JETIEX_TYPE_TIMEDATE 5
#define JETIEX_TYPE_INT30 8
#define JETIEX_TYPE_COORDINATES 9

#define JETIEX_FORMAT_0_DECIMAL 0
#define JETIEX_FORMAT_1_DECIMAL 1
#define JETIEX_FORMAT_2_DECIMAL 2
#define JETIEX_FORMAT_3_DECIMAL 3
#define JETIEX_FORMAT_DATE 1
#define JETIEX_FORMAT_LON 1
#define JETIEX_FORMAT_TIME 0
#define JETIEX_FORMAT_LAT 0

#define JETIEX_MFG_ID_LOW 0x00
#define JETIEX_MFG_ID_HIGH 0xA4
#define JETIEX_DEV_ID_LOW 0x00
#define JETIEX_DEV_ID_HIGH 0xA4

#define JETIEX_WAIT 0
#define JETIEX_SEND 1

#define JETIEX_PACKET_LENGHT 8
#define JETIEX_TIMEOUT_US 500
#define JETIEX_BAUDRATE_TIMEOUT_MS 5000

typedef struct sensor_jetiex_t
{
    uint8_t data_id;
    uint8_t type;
    uint8_t format;
    char text[32];
    char unit[8];
    float *value;

} sensor_jetiex_t;

extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;
extern TaskHandle_t pwm_out_task_handle, led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;
extern uint8_t debug;

void jetiex_task(void *parameters);

#endif