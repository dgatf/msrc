#ifndef SRXL_H
#define SRXL_H

#include "common.h"
#include "xbus.h"

extern context_t context;
extern xbus_sensor_t *sensor;
extern xbus_sensor_formatted_t *sensor_formatted;

void srxl_task(void *parameters);

#endif