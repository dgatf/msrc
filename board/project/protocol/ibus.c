#include "ibus.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "airspeed.h"
#include "bmp180.h"
#include "bmp280.h"
#include "config.h"
#include "current.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "esc_hw5.h"
#include "esc_kontronik.h"
#include "esc_omp_m4.h"
#include "esc_openyge.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "gps.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

/* Flysky IBUS Data Id */
#define IBUS_ID_VOLTAGE 0x00       // Internal Voltage
#define IBUS_ID_TEMPERATURE 0x01   // Temperature
#define IBUS_ID_MOT 0x02           // RPM
#define IBUS_ID_EXTV 0x03          // External Voltage
#define IBUS_ID_CELL_VOLTAGE 0x04  // Avg Cell voltage
#define IBUS_ID_BAT_CURR 0x05      // battery current A * 100
#define IBUS_ID_FUEL 0x06          // remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define IBUS_ID_RPM 0x07           // throttle value / battery capacity
#define IBUS_ID_CMP_HEAD 0x08      // Heading  0..360 deg 0north 2bytes
#define IBUS_ID_CLIMB_RATE 0x09    // 2 bytes m/s *100 signed
#define IBUS_ID_COG \
    0x0A  // 2 bytes  Course over ground(NOT heading but direction of movement) in degrees * 100 0.0..359.99 degrees.
          // unknown max uint
#define IBUS_ID_GPS_STATUS 0x0B      // 2 bytes
#define IBUS_ID_ACC_X 0x0C           // 2 bytes m/s *100 signed
#define IBUS_ID_ACC_Y 0x0D           // 2 bytes m/s *100 signed
#define IBUS_ID_ACC_Z 0x0E           // 2 bytes m/s *100 signed
#define IBUS_ID_ROLL 0x0F            // 2 bytes deg *100 signed
#define IBUS_ID_PITCH 0x10           // 2 bytes deg *100 signed
#define IBUS_ID_YAW 0x11             // 2 bytes deg *100 signed
#define IBUS_ID_VERTICAL_SPEED 0x12  // 2 bytes m/s *100 signed
#define IBUS_ID_GROUND_SPEED 0x13    // 2 bytes m/s *100 different unit than build-in sensor
#define IBUS_ID_GPS_DIST 0x14        // 2 bytes distance from home m unsigned
#define IBUS_ID_ARMED 0x15           // 2 bytes
#define IBUS_ID_FLIGHT_MODE 0x16     // 2 bytes
#define IBUS_ID_PRES 0x41            // Pressure
#define IBUS_ID_ODO1 0x7C            // Odometer1
#define IBUS_ID_ODO2 0x7D            // Odometer2
#define IBUS_ID_SPE 0x7E             // Speed 2 bytes km/h * 100
#define IBUS_ID_TX_V 0x7F            // TX Voltage
#define IBUS_ID_GPS_LAT 0x80         // 4bytes signed WGS84 in degrees * 1E7
#define IBUS_ID_GPS_LON 0x81         // 4bytes signed WGS84 in degrees * 1E7
#define IBUS_ID_GPS_ALT 0x82         // 4bytes signed!!! GPS alt m*100
#define IBUS_ID_ALT 0x83             // 4bytes signed!!! Alt m*100
#define IBUS_ID_S84 0x84
#define IBUS_ID_S85 0x85
#define IBUS_ID_S86 0x86
#define IBUS_ID_S87 0x87
#define IBUS_ID_S88 0x88
#define IBUS_ID_S89 0x89
#define IBUS_ID_S8a 0x8A
#define IBUS_ID_RX_SIG_AFHDS3 0xF7  // SIG
#define IBUS_ID_RX_SNR_AFHDS3 0xF8  // SNR
#define IBUS_ID_ALT_FLYSKY 0xF9     // Altitude 2 bytes signed in m - used in FlySky native TX
#define IBUS_ID_RX_SNR 0xFA         // SNR
#define IBUS_ID_RX_NOISE 0xFB       // Noise
#define IBUS_ID_RX_RSSI 0xFC        // RSSI
#define IBUS_ID_RX_ERR_RATE 0xFE    // Error rate
#define IBUS_ID_END 0xFF
// AC type telemetry with multiple values in one packet
#define IBUS_ID_GPS_FULL 0xFD
#define IBUS_ID_VOLT_FULL 0xF0
#define IBUS_ID_ACC_FULL 0xEF
#define IBUS_ID_TX_RSSI 0x200  // Pseudo id outside 1 byte range of FlySky sensors

#define IBUS_TYPE_U16 0
#define IBUS_TYPE_S16 1
#define IBUS_TYPE_U32 2
#define IBUS_TYPE_S32 3
#define IBUS_TYPE_GPS 4

#define IBUS_RECEIVED_NONE 0
#define IBUS_RECEIVED_POLL 1

#define IBUS_COMMAND_DISCOVER 0x8
#define IBUS_COMMAND_TYPE 0x9
#define IBUS_COMMAND_MEASURE 0xA

#define IBUS_TIMEOUT_US 1000
#define IBUS_PACKET_LENGHT 4

typedef struct sensor_ibus_t {
    uint8_t data_id;
    uint8_t type;
    float *value;
} sensor_ibus_t;

static void process(sensor_ibus_t **sensor);
static void send_packet(uint8_t command, uint8_t address, sensor_ibus_t *sensor_ibus);
static void send_byte(uint8_t c, uint16_t *crc_p);
static int32_t format(uint8_t data_id, float value);
static bool check_crc(uint8_t *data);
static void add_sensor(sensor_ibus_t *new_sensor, sensor_ibus_t **sensor, uint16_t sensormask);
static void set_config(sensor_ibus_t **sensor, uint16_t sensormask);

void ibus_task(void *parameters) {
    sensor_ibus_t *sensor[16] = {NULL};
    uint16_t sensor_mask = 0B1111111111111110;
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(115200, UART_RECEIVER_TX, UART_RECEIVER_RX, IBUS_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    set_config(sensor, sensor_mask);
    debug("\nIbus init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(sensor);
    }
}

static void process(sensor_ibus_t **sensor) {
    if (uart0_available() == IBUS_PACKET_LENGHT) {
        uint8_t command = 0;
        uint8_t address = 0;
        uint8_t data[IBUS_PACKET_LENGHT];
        uart0_read_bytes(data, IBUS_PACKET_LENGHT);
        if (data[0] == IBUS_PACKET_LENGHT) {
            debug("\nIbus (%u) < ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(data, data[0], "0x%X ");
            if (check_crc(data)) {
                command = data[1] >> 4;
                address = data[1] & 0x0F;
                if (!sensor[address]) return;
                if (command == IBUS_COMMAND_DISCOVER || command == IBUS_COMMAND_TYPE || IBUS_COMMAND_MEASURE)
                    send_packet(command, address, sensor[address]);
            } else
                debug(" - Bad CRC");
        }
    }
}

static void send_packet(uint8_t command, uint8_t address, sensor_ibus_t *sensor) {
    uint8_t *u8_p = NULL;
    uint16_t crc = 0;
    uint16_t type;
    uint8_t lenght = 0;
    int32_t value_formatted = 0;

    switch (sensor->type) {
        case IBUS_TYPE_S16:
        case IBUS_TYPE_U16:
            lenght = 2;
            break;
        case IBUS_TYPE_S32:
            lenght = 4;
            break;
        case IBUS_TYPE_GPS:
            lenght = 14;
            break;
    }
    switch (command) {
        case IBUS_COMMAND_DISCOVER:
            lenght = 0;
            break;
        case IBUS_COMMAND_TYPE:
            type = lenght << 8 | sensor->data_id;
            u8_p = (uint8_t *)&type;
            lenght = 2;
            break;
        case IBUS_COMMAND_MEASURE:
            if (sensor->value) {
                value_formatted = format(sensor->data_id, *sensor->value);
            }
            u8_p = (uint8_t *)&value_formatted;
            break;
    }
    debug("\nIbus (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    // lenght
    send_byte(4 + lenght, &crc);

    // command & address
    send_byte(command << 4 | address, &crc);

    // value
    for (uint8_t i = 0; i < lenght; i++) {
        send_byte(u8_p[i], &crc);
    }

    // crc
    crc = 0xFFFF - crc;
    u8_p = (uint8_t *)&crc;
    send_byte(u8_p[0], NULL);
    send_byte(u8_p[1], NULL);

    // blink led
    vTaskResume(context.led_task_handle);
}

static void send_byte(uint8_t c, uint16_t *crc_p) {
    if (crc_p != NULL) {
        uint16_t crc = *crc_p;
        crc += c;
        *crc_p = crc;
    }
    uart0_write(c);
    debug("%X ", c);
}

static bool check_crc(uint8_t *data) {
    uint16_t crc = 0xFFFF;
    uint8_t lenght = data[0];
    for (uint8_t i = 0; i < lenght - 2; i++) crc -= data[i];
    if (crc == (uint16_t)data[lenght - 2] << 8 || data[lenght - 1]) return true;
    return false;
}

static void add_sensor(sensor_ibus_t *new_sensor, sensor_ibus_t **sensor, uint16_t sensor_mask) {
    for (uint8_t i = 0; i < 16; i++) {
        if (sensor[i] == NULL && sensor_mask & 1 << i) {
            sensor[i] = new_sensor;
            return;
        }
    }
}

static int32_t format(uint8_t data_id, float value) {
    if (data_id == IBUS_ID_TEMPERATURE) return round((value + 40) * 10);

    if (data_id == IBUS_ID_EXTV || data_id == IBUS_ID_CELL_VOLTAGE || data_id == IBUS_ID_BAT_CURR ||
        data_id == IBUS_ID_CLIMB_RATE || data_id == IBUS_ID_COG || data_id == IBUS_ID_VERTICAL_SPEED ||
        data_id == IBUS_ID_GROUND_SPEED || data_id == IBUS_ID_GPS_ALT || data_id == IBUS_ID_PRES ||
        data_id == IBUS_ID_ALT || data_id == IBUS_ID_ACC_X || data_id == IBUS_ID_ACC_Y || data_id == IBUS_ID_ACC_Z ||
        data_id == IBUS_ID_ROLL || data_id == IBUS_ID_PITCH || data_id == IBUS_ID_YAW)
        return round(value * 100);

    if (data_id == IBUS_ID_GPS_LAT || data_id == IBUS_ID_GPS_LON) return round(value * 1e7);

    if (data_id == IBUS_ID_SPE) return round(value * 100 * 1.852);

    if (data_id == IBUS_ID_GPS_STATUS) return value * 256;

    if (data_id == IBUS_ID_S84 || data_id == IBUS_ID_S85) return value / 60 * 1e5;

    return round(value);
}

static void set_config(sensor_ibus_t **sensor, uint16_t sensormask) {
    /*
    - Sensor at address 0x00 is reserved
    - Sensor at address 0x01 is recerved in some receivers types. But the poll at address 0x01, if present, has to be
    answered (so there is a dummy sensor at address 0x01), otherwise the sensor poll scan is stopped from receiver
    - TODO: dinamically set the sensor address to allocate an additional sensor in receivers with only one sensor masked
   */
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_ibus_t *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;
    new_sensor = malloc(sizeof(sensor_ibus_t));
    *new_sensor = (sensor_ibus_t){IBUS_ID_END, 0, NULL};
    add_sensor(new_sensor, sensor, sensormask);
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);

        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW4) {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier,
                                          config->enable_pwm_out,
                                          config->enable_esc_hw4_init_delay,
                                          config->alpha_rpm,
                                          config->alpha_voltage,
                                          config->alpha_current,
                                          config->alpha_temperature,
                                          config->esc_hw4_voltage_multiplier,
                                          config->esc_hw4_current_multiplier,
                                          config->esc_hw4_current_thresold,
                                          config->esc_hw4_current_max,
                                          config->esc_hw4_is_manual_offset,
                                          config->esc_hw4_auto_detect,
                                          config->esc_hw4_offset,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_hw4_task", STACK_ESC_HW4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_pwm_out) {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            context.pwm_out_task_handle = task_handle;
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
    }
    if (config->esc_protocol == ESC_HW5) {
        esc_hw5_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw5_task, "esc_hw5_task", STACK_ESC_HW5, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
    }
    if (config->esc_protocol == ESC_CASTLE) {
        esc_castle_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                             config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.ripple_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current_bec};
        add_sensor(new_sensor, sensor, sensormask);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_KONTRONIK) {
        esc_kontronik_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage,  config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_kontronik_task, "esc_kontronik_task", STACK_ESC_KONTRONIK, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_F) {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                            config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_HV) {
        esc_apd_hv_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_hv_task, "esc_apd_hv_task", STACK_ESC_APD_HV, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_SMART) {
        smart_esc_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temperature_fet = malloc(sizeof(float));
        parameter.temperature_bec = malloc(sizeof(float));
        parameter.voltage_bec = malloc(sizeof(float));
        parameter.current_bec = malloc(sizeof(float));
        parameter.temperature_bat = malloc(sizeof(float));
        parameter.current_bat = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        for (uint i = 0; i < 18; i++) parameter.cell[i] = malloc(sizeof(float));
        parameter.cells = malloc(sizeof(uint8_t));
        parameter.cycles = malloc(sizeof(uint16_t));
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 4, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_OMP_M4) {
        esc_omp_m4_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_omp_m4_task, "esc_omp_m4_task", STACK_ESC_OMP_M4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temp_esc};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temp_motor};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_ZTW) {
        esc_ztw_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.bec_voltage = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_ztw_task, "esc_ztw_task", STACK_ESC_ZTW, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temp_esc};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temp_motor};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_OPENYGE) {
        esc_openyge_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.pwm_out = config->enable_pwm_out;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temperature_fet = malloc(sizeof(float));
        parameter.temperature_bec = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.voltage_bec = malloc(sizeof(float));
        parameter.current_bec = malloc(sizeof(float));
        parameter.throttle = malloc(sizeof(float));
        parameter.pwm_percent = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_openyge_task, "esc_openyge_task", STACK_ESC_OPENYGE, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps) {
        gps_parameters_t parameter;
        parameter.protocol = config->gps_protocol;
        parameter.baudrate = config->gps_baudrate;
        parameter.rate = config->gps_rate;
        parameter.lat = malloc(sizeof(double));
        parameter.lon = malloc(sizeof(double));
        parameter.alt = malloc(sizeof(float));
        parameter.spd = malloc(sizeof(float));
        parameter.cog = malloc(sizeof(float));
        parameter.hdop = malloc(sizeof(float));
        parameter.sat = malloc(sizeof(float));
        parameter.time = malloc(sizeof(float));
        parameter.date = malloc(sizeof(float));
        parameter.vspeed = malloc(sizeof(float));
        parameter.dist = malloc(sizeof(float));
        parameter.spd_kmh = malloc(sizeof(float));
        parameter.fix = malloc(sizeof(float));
        parameter.vdop = malloc(sizeof(float));
        parameter.speed_acc = malloc(sizeof(float));
        parameter.h_acc = malloc(sizeof(float));
        parameter.v_acc = malloc(sizeof(float));
        parameter.track_acc = malloc(sizeof(float));
        parameter.n_vel = malloc(sizeof(float));
        parameter.e_vel = malloc(sizeof(float));
        parameter.v_vel = malloc(sizeof(float));
        parameter.alt_elipsiod = malloc(sizeof(float));
        parameter.pdop = malloc(sizeof(float));
        xTaskCreate(gps_task, "gps_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_GPS_STATUS, IBUS_TYPE_U16, parameter.sat};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_GPS_LAT, IBUS_TYPE_S32, parameter.lat};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_GPS_LON, IBUS_TYPE_S32, parameter.lon};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_GPS_ALT, IBUS_TYPE_S32, parameter.alt};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_SPE, IBUS_TYPE_U16, parameter.spd};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_COG, IBUS_TYPE_U16, parameter.cog};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_GPS_DIST, IBUS_TYPE_U16, parameter.dist};
        add_sensor(new_sensor, sensor, sensormask);
        if (config->ibus_alternative_coordinates) {
            new_sensor = malloc(sizeof(sensor_ibus_t));
            *new_sensor = (sensor_ibus_t){IBUS_ID_S84, IBUS_TYPE_S32, parameter.lat};
            add_sensor(new_sensor, sensor, sensormask);
            new_sensor = malloc(sizeof(sensor_ibus_t));
            *new_sensor = (sensor_ibus_t){IBUS_ID_S85, IBUS_TYPE_S32, parameter.lon};
            add_sensor(new_sensor, sensor, sensormask);
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current) {
        current_parameters_t parameter = {1,
                                          config->analog_rate,
                                          config->alpha_current,
                                          config->analog_current_multiplier,
                                          config->analog_current_offset,
                                          config->analog_current_autoffset,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.ntc};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ALT, IBUS_TYPE_S32, parameter.altitude};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ALT, IBUS_TYPE_S32, parameter.altitude};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ALT, IBUS_TYPE_S32, parameter.altitude};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 1000,
                                           (float)config->airspeed_vcc / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_SPE, IBUS_TYPE_U16, parameter.airspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gyro) {
        mpu6050_parameters_t parameter = {1,
                                          0,
                                          config->mpu6050_acc_scale,
                                          config->mpu6050_gyro_scale,
                                          config->mpu6050_gyro_weighting,
                                          config->mpu6050_filter,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float))};
        xTaskCreate(mpu6050_task, "mpu6050_task", STACK_MPU6050, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ACC_X, IBUS_TYPE_S16, parameter.acc_x};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ACC_Y, IBUS_TYPE_S16, parameter.acc_y};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ACC_Z, IBUS_TYPE_S16, parameter.acc_z};
        add_sensor(new_sensor, sensor, sensormask);

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_PITCH, IBUS_TYPE_S16, parameter.pitch};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_ROLL, IBUS_TYPE_S16, parameter.roll};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){IBUS_ID_YAW, IBUS_TYPE_S16, parameter.yaw};
        add_sensor(new_sensor, sensor, sensormask);
    }
}
