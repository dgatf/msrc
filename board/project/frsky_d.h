#ifndef FRSKY_D_H
#define FRSKY_D_H

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

/* FrSky D Data Id */
#define FRSKY_D_GPS_ALT_BP_ID 0x01
#define FRSKY_D_TEMP1_ID 0x02
#define FRSKY_D_RPM_ID 0x03
#define FRSKY_D_FUEL_ID 0x04
#define FRSKY_D_TEMP2_ID 0x05
#define FRSKY_D_CELL_VOLT_ID 0x06
#define FRSKY_D_GPS_ALT_AP_ID 0x09
#define FRSKY_D_BARO_ALT_BP_ID 0x10
#define FRSKY_D_GPS_SPEED_BP_ID 0x11
#define FRSKY_D_GPS_LONG_BP_ID 0x12
#define FRSKY_D_GPS_LAT_BP_ID 0x13
#define FRSKY_D_GPS_COURS_BP_ID 0x14
#define FRSKY_D_GPS_DAY_MONTH_ID 0x15
#define FRSKY_D_GPS_YEAR_ID 0x16
#define FRSKY_D_GPS_HOUR_MIN_ID 0x17
#define FRSKY_D_GPS_SEC_ID 0x18
#define FRSKY_D_GPS_SPEED_AP_ID 0x19
#define FRSKY_D_GPS_LONG_AP_ID 0x1A
#define FRSKY_D_GPS_LAT_AP_ID 0x1B
#define FRSKY_D_GPS_COURS_AP_ID 0x1C
#define FRSKY_D_BARO_ALT_AP_ID 0x21
#define FRSKY_D_GPS_LONG_EW_ID 0x22
#define FRSKY_D_GPS_LAT_NS_ID 0x23
#define FRSKY_D_ACCEL_X_ID 0x24
#define FRSKY_D_ACCEL_Y_ID 0x25
#define FRSKY_D_ACCEL_Z_ID 0x26
#define FRSKY_D_CURRENT_ID 0x28
#define FRSKY_D_VARIO_ID 0x30
#define FRSKY_D_VFAS_ID 0x39
#define FRSKY_D_VOLTS_BP_ID 0x3A
#define FRSKY_D_VOLTS_AP_ID 0x3B
#define FRSKY_D_FRSKY_LAST_ID 0x3F
#define FRSKY_D_D_RSSI_ID 0xF0
#define FRSKY_D_D_A1_ID 0xF1
#define FRSKY_D_D_A2_ID 0xF2

#define FRSKY_D_INTERVAL 10

typedef struct frsky_d_sensor_parameters_t
{
    uint8_t data_id;
    float *value;
    uint16_t rate;

} frsky_d_sensor_parameters_t;

typedef struct frsky_d_sensor_cell_parameters_t
{
    float *voltage;
    uint8_t *count;
    uint16_t rate;

} frsky_d_sensor_cell_parameters_t;

extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;
extern TaskHandle_t pwm_out_task_handle, led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;

void frsky_d_task(void *parameters);

#endif