#ifndef GPS_H
#define GPS_H

#include "common.h"

typedef struct gps_parameters_t {
    gps_protocol_t protocol;
    uint baudrate, rate;
    double *lat, *lon;
    float *alt, *spd, *cog, *hdop, *sat, *time, *date, *vspeed, *dist, *spd_kmh, *fix, *vdop, *speed_acc, *h_acc,
        *v_acc, *track_acc, *n_vel, *e_vel, *v_vel, *alt_elipsiod;
} gps_parameters_t;

extern context_t context;

void gps_task(void *parameters);

#endif