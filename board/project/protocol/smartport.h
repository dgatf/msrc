#ifndef SMARTPORT_H
#define SMARTPORT_H

#include "common.h"

typedef enum coordinate_type_t {
    SMARTPORT_LATITUDE,
    SMARTPORT_LONGITUDE,
} coordinate_type_t;

typedef enum datetime_type_t {
    SMARTPORT_DATE,
    SMARTPORT_TIME,
} datetime_type_t;

typedef struct smartport_parameters_t {
    uint8_t sensor_id;
    uint16_t data_id;
} smartport_parameters_t;

typedef struct smartport_sensor_parameters_t {
    uint16_t data_id;
    float *value;
    uint16_t rate;
} smartport_sensor_parameters_t;

typedef struct smartport_sensor_gpio_parameters_t {
    uint16_t data_id;
    uint8_t gpio_mask;
    uint8_t *value;
    uint16_t rate;
} smartport_sensor_gpio_parameters_t;

typedef struct smartport_sensor_double_parameters_t {
    uint16_t data_id;
    float *value_l;
    float *value_h;
    uint16_t rate;
} smartport_sensor_double_parameters_t;

typedef struct smartport_sensor_coordinate_parameters_t {
    coordinate_type_t type;
    float *latitude;
    float *longitude;
    uint16_t rate;
} smartport_sensor_coordinate_parameters_t;

typedef struct smartport_sensor_datetime_parameters_t {
    datetime_type_t type;
    float *date;
    float *time;
    uint16_t rate;
} smartport_sensor_datetime_parameters_t;

typedef struct smartport_sensor_cell_parameters_t {
    uint8_t *cell_count;
    float *cell_voltage;
    uint16_t rate;
} smartport_sensor_cell_parameters_t;

typedef struct smartport_packet_parameters_t {
    uint16_t data_id;
    QueueHandle_t queue_handle;
} smartport_packet_parameters_t;

typedef struct smartport_packet_t {
    uint16_t type_id;
    uint16_t data_id;
    uint32_t data;
} smartport_packet_t;

extern context_t context;

void smartport_task(void *parameters);

#endif