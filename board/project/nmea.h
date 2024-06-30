#ifndef NMEA_H
#define NMEA_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"

#include "uart_pio.h"
#include "common.h"
#include "vspeed.h"
#include "distance.h"
#include "config.h"

#define NMEA_GGA 0
#define NMEA_RMC 1
#define NMEA_UNK 2
#define NMEA_GSA 3

#define NMEA_VTG 4
#define NMEA_GLL 5

#define NMEA_LON 1
#define NMEA_ALT 2
#define NMEA_SPD 3
#define NMEA_COG 4
#define NMEA_FIX 5
#define NMEA_SAT 6
#define NMEA_DATE 7
#define NMEA_TIME 8
#define NMEA_LAT_SIGN 9
#define NMEA_LON_SIGN 10
#define NMEA_HDOP 11
#define NMEA_END 12
#define NMEA_LAT 13

#define NMEA_TIMEOUT_US 5000

typedef struct nmea_parameters_t
{
    uint32_t baudrate;
    float *lat, *lon, *alt, *spd, *cog, *hdop, *sat, *time, *date, *vspeed, *dist, *spd_kmh;
} nmea_parameters_t;

extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;
extern TaskHandle_t receiver_task_handle;

void nmea_task(void *parameters);

#endif