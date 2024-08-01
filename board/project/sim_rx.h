#ifndef SIM_RX_H
#define SIM_RX_H

#include "common.h"

extern context_t context;

typedef struct sim_rx_parameters_t {
    rx_protocol_t rx_protocol;
} sim_rx_parameters_t;

void sim_rx_task(void *parameters);

#endif