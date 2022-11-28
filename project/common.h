#ifndef COMMON_H
#define COMMON_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

#include "constants.h"

float get_average(float alpha, float prev_value, float new_value);
float get_consumption(float current, uint16_t current_max, uint32_t *timestamp);
float voltage_read(uint8_t adc_num);
float get_altitude(float pressure, float temperature, float P0);

/*
typedef struct buffer_node_t
{
    void *item;
    void *next;
} buffer_node_t;

void circular_buffer_add(buffer_node_t *node, void *item);
void circular_buffer_next(buffer_node_t *node);
void circular_buffer_empty(buffer_node_t *node);
*/

#endif