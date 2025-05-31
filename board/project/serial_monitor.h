#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

#include "common.h"

typedef struct serial_monitor_parameters_t {
    uint8_t gpio;
    uint32_t baudrate;
    uint8_t stop_bits;
    uint16_t timeout_ms;
    uint8_t parity;
    bool inverted;
} serial_monitor_parameters_t;

extern context_t context;

void serial_monitor_task(void *parameters);

#endif