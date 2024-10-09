#ifndef SRXL_H
#define SRXL_H

#include "common.h"
#include "xbus.h"

extern context_t context;
extern xbus_sensor_t *sensor;
extern xbus_sensor_formatted_t *sensor_formatted;

void srxl_task(void *parameters);
uint16_t get_crc(uint8_t *buffer, uint8_t length);
uint16_t crc16(uint16_t crc, uint8_t data);

#endif