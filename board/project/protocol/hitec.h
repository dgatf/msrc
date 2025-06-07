#ifndef HITEC_H
#define HITEC_H

#include "common.h"

extern context_t context;

void hitec_task(void *parameters);
void hitec_i2c_handler(void);

#endif