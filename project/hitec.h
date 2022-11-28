#ifndef HITEC_H
#define HITEC_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
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

#define HITEC_I2C_ADDRESS 0x08
#define HITEC_TIMEOUT 1000

#define HITEC_FRAME_0X11 0
#define HITEC_FRAME_0X12 1
#define HITEC_FRAME_0X13 2
#define HITEC_FRAME_0X14 3
#define HITEC_FRAME_0X15 4
#define HITEC_FRAME_0X16 5
#define HITEC_FRAME_0X17 6
#define HITEC_FRAME_0X18 7
#define HITEC_FRAME_0X19 8
#define HITEC_FRAME_0X1A 9
#define HITEC_FRAME_0X1B 10

#define HITEC_FRAME_0X11_RX_BATT 0

#define HITEC_FRAME_0X12_GPS_LAT 0
#define HITEC_FRAME_0X12_TIME 1

#define HITEC_FRAME_0X13_GPS_LON 0
#define HITEC_FRAME_0X13_TEMP2 1

#define HITEC_FRAME_0X14_GPS_SPD 0
#define HITEC_FRAME_0X14_GPS_ALT 1
#define HITEC_FRAME_0X14_TEMP1 2

#define HITEC_FRAME_0X15_FUEL 0
#define HITEC_FRAME_0X15_RPM1 1
#define HITEC_FRAME_0X15_RPM2 2

#define HITEC_FRAME_0X16_DATE 0
#define HITEC_FRAME_0X16_TIME 1

#define HITEC_FRAME_0X17_COG 0
#define HITEC_FRAME_0X17_SATS 1
#define HITEC_FRAME_0X17_TEMP3 2
#define HITEC_FRAME_0X17_TEMP4 3

#define HITEC_FRAME_0X18_VOLT 0
#define HITEC_FRAME_0X18_AMP 1

#define HITEC_FRAME_0X19_AMP1 0
#define HITEC_FRAME_0X19_AMP2 1
#define HITEC_FRAME_0X19_AMP3 2
#define HITEC_FRAME_0X19_AMP4 3

#define HITEC_FRAME_0X1A_ASPD 0

#define HITEC_FRAME_0X1B_ALTU 0
#define HITEC_FRAME_0X1B_ALTF 1

typedef struct sensor_hitec_t
{
    bool is_enabled_frame[11];
    float *frame_0x11[1];
    float *frame_0x12[2];
    float *frame_0x13[2];
    float *frame_0x14[3];
    float *frame_0x15[3];
    float *frame_0x16[2];
    float *frame_0x17[4];
    float *frame_0x18[2];
    float *frame_0x19[4];
    float *frame_0x1A[1];
    float *frame_0x1B[2];
} sensor_hitec_t;

extern QueueHandle_t tasks_queue_handle;
extern TaskHandle_t pwm_out_task_handle, led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;

void hitec_task(void *parameters);

#ifdef SIM_RX
void hitec_i2c_handler();
#endif

#endif