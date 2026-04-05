#ifndef NTC_H
#define NTC_H

#include "common.h"

typedef struct ntc_parameters_t {
    uint8_t adc_num;
    uint8_t rate;
    float alpha;
    int8_t offset;
    float *ntc;

} ntc_parameters_t;

extern context_t context;

void ntc_task(void *parameters);

#endif