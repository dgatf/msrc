#ifndef JETIEX_H
#define JETIEX_H

#include "common.h"

#define JETIEX_TYPE_INT6 0
#define JETIEX_TYPE_INT14 1
#define JETIEX_TYPE_INT22 4
#define JETIEX_TYPE_TIMEDATE 5
#define JETIEX_TYPE_INT30 8
#define JETIEX_TYPE_COORDINATES 9

#define JETIEX_FORMAT_0_DECIMAL 0
#define JETIEX_FORMAT_1_DECIMAL 1
#define JETIEX_FORMAT_2_DECIMAL 2
#define JETIEX_FORMAT_3_DECIMAL 3
#define JETIEX_FORMAT_DATE 1
#define JETIEX_FORMAT_LON 1
#define JETIEX_FORMAT_TIME 0
#define JETIEX_FORMAT_LAT 0

#define JETIEX_MFG_ID_LOW 0x00
#define JETIEX_MFG_ID_HIGH 0xA4
#define JETIEX_DEV_ID_LOW 0x00
#define JETIEX_DEV_ID_HIGH 0xA4

typedef struct sensor_jetiex_t {
    uint8_t data_id;
    uint8_t type;
    uint8_t format;
    char text[32];
    char unit[8];
    float *value;
} sensor_jetiex_t;

extern context_t context;

void jetiex_task(void *parameters);
void jeti_add_sensor(sensor_jetiex_t *new_sensor, sensor_jetiex_t **sensors);
uint8_t jeti_create_telemetry_buffer(uint8_t *buffer, bool packet_type, sensor_jetiex_t **sensor);
void jeti_set_config(sensor_jetiex_t **sensor);

#endif