#ifndef NMEA_H
#define NMEA_H

#include "common.h"

typedef struct nmea_parameters_t {
    uint32_t baudrate;
    float *lat, *lon, *alt, *spd, *cog, *hdop, *sat, *time, *date, *vspeed, *dist, *spd_kmh;
} nmea_parameters_t;

extern context_t context;

void nmea_task(void *parameters);

#endif