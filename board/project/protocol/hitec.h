#ifndef HITEC_H
#define HITEC_H

#include "common.h"

typedef struct sensor_hitec_t {
    bool is_enabled_frame[11];
    float *frame_0x11[1];
    float *frame_0x12[2];
    float *frame_0x13[2];
    float *frame_0x14[3];
    float *frame_0x15[3];
    float *frame_0x16[2];
    float *frame_0x17[4];
    float *frame_0x18[2];
    float *frame_0x19[4];
    float *frame_0x1A[1];
    float *frame_0x1B[2];
} sensor_hitec_t;

extern context_t context;

void hitec_task(void *parameters);

#endif