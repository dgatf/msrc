#ifndef SRXL2_H
#define SRXL2_H

#include "common.h"
#include "xbus.h"

extern context_t context;
extern xbus_sensor_t *sensor;
extern xbus_sensor_formatted_t *sensor_formatted;

void srxl2_task(void *parameters);

#endif