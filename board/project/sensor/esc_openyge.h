#ifndef ESC_OPENYGE_H
#define ESC_OPENYGE_H

#include "common.h"

typedef enum {
    CRC_MODE_UNKNOWN = 0,
    CRC_MODE_INC_SYNC_SEED_FFFF_BE,
    CRC_MODE_INC_SYNC_SEED_FFFF_LE,
    CRC_MODE_SKIP_SYNC_SEED_FFFF_BE,
    CRC_MODE_SKIP_SYNC_SEED_FFFF_LE,
    CRC_MODE_INC_SYNC_SEED_0000_BE,
    CRC_MODE_INC_SYNC_SEED_0000_LE,
    CRC_MODE_SKIP_SYNC_SEED_0000_BE,
    CRC_MODE_SKIP_SYNC_SEED_0000_LE
} crc_mode_t;

typedef struct esc_openyge_parameters_t {
    float rpm_multiplier;
    bool pwm_out;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *cell_voltage, *consumption;
    float *voltage_bec, *current_bec, *throttle, *pwm_percent;
    uint8_t *cell_count;
    uint32_t crc_errors;
    crc_mode_t crc_mode;
} esc_openyge_parameters_t;

extern context_t context;

void esc_openyge_task(void *parameters);

#endif