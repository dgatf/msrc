#ifndef GPIO_SENSOR_H
#define GPIO_SENSOR_H

#include "common.h"

typedef struct gpio_parameters_t {
    uint16_t interval;
    float *value;
} gpio_parameters_t;

extern context_t context;

void gpio_task(void *parameters);

#endif