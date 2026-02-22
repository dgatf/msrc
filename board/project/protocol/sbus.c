#include "sbus.h"

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
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define SLOT_TEMP1 1
#define SLOT_RPM 2
#define SLOT_VARIO_SPEED 3
#define SLOT_VARIO_ALT 4
#define SLOT_VARIO_RESSURE 5
#define SLOT_VOLT_V1 6
#define SLOT_VOLT_V2 7
#define SLOT_GPS_SPD 8
#define SLOT_GPS_ALT 9
#define SLOT_GPS_TIME 10
#define SLOT_GPS_VARIO 11
#define SLOT_GPS_LAT1 12
#define SLOT_GPS_LAT2 13
#define SLOT_GPS_LON1 14
#define SLOT_GPS_LON2 15
#define SLOT_AIR_SPEED 16
#define SLOT_POWER_CURR3 21
#define SLOT_POWER_VOLT3 22
#define SLOT_POWER_CONS3 23
#define SLOT_POWER_CURR1 24
#define SLOT_POWER_VOLT1 25
#define SLOT_POWER_CONS1 26
#define SLOT_POWER_CURR2 27
#define SLOT_POWER_VOLT2 28
#define SLOT_POWER_CONS2 29
#define SLOT_TEMP2 30

#define TIMEOUT_US 500
#define SLOT_0_DELAY 1500
#define INTER_SLOT_DELAY 700
#define PACKET_LENGHT 25
#define SBUS_NEGATIVE_BIT 15
#define SBUS_SOUTH_WEST_BIT 4
#define SBUS_WAIT 0
#define SBUS_SEND 1

/* FASST Sbus Data Id */
#define SBUS_NULL 0
#define SBUS_TEMP 1
#define SBUS_VOLT_V1 2
#define SBUS_VOLT_V2 3
#define SBUS_RPM 4
#define SBUS_POWER_CURR 5
#define SBUS_POWER_VOLT 6
#define SBUS_POWER_CONS 7
#define SBUS_VARIO_ALT 8  // F1672
#define SBUS_VARIO_SPEED 9
#define SBUS_GPS_SPEED 10  // F1675
#define SBUS_GPS_ALTITUDE 11
#define SBUS_GPS_TIME 12
#define SBUS_GPS_VARIO_SPEED 13
#define SBUS_GPS_LATITUDE1 14
#define SBUS_GPS_LATITUDE2 15
#define SBUS_GPS_LONGITUDE1 16
#define SBUS_GPS_LONGITUDE2 17
#define SBUS_AIR_SPEED 18

/**
 * Slots sensor mapping for Futaba transmitters:
 *
 * | Slot   | Sensor                               |
 * | :----: | :----------------------------------: |
 * |0       | RX voltage (reserved)                |
 * |1       | Temperature 1 (SBS-01T/TE)           |
 * |2       | RPM (type magnet)(SBS-01RB/RM/RO)    |
 * |3-4     | Vario-F1672                          |
 * |6-7     | Voltage (SBS-01V)                    |
 * |8-15    | GPS-F1675                            |
 * |16      | Air speed (SBS-01TAS)                |
 * |17-21   | Unused                               |
 * |21-23   | Current 3 (SBS-01C)                  |
 * |24-26   | Current 1 (SBS-01C)                  |
 * |27-29(+)| Current 2 (SBS-01C)                  |
 * |30(+)   | Temperature 2 (SBS-01T/TE)           |
 * |31      | Unused                               |
 *
 * (+) Non default slots
 */

typedef struct sensor_sbus_t {
    uint8_t data_id;
    float *value;
} sensor_sbus_t;

static sensor_sbus_t *sbus_sensor[32] = {NULL};
static uint packet_id;
static volatile uint slot = 0;
static volatile bool slot_pending = false;

static void process();
static int64_t send_slot_callback(alarm_id_t id, void *parameters);
static inline void send_slot(uint8_t slot);
static uint16_t format(uint8_t data_id, float value);
static void add_sensor(uint8_t slot, sensor_sbus_t *new_sensor);
static void set_config(void);
static uint8_t get_slot_id(uint8_t slot);

void sbus_task(void *parameters) {
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    set_config();
    uart0_begin(100000, UART_RECEIVER_TX, UART_RECEIVER_RX, TIMEOUT_US, 8, 2, UART_PARITY_EVEN, true, true);
    debug("\nSbus init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
        if (slot_pending) {
            send_slot(slot);
            slot_pending = false;
        }
    }
}

static void process() {
    if (uart0_available() == PACKET_LENGHT) {
        uint8_t data[PACKET_LENGHT];
        uart0_read_bytes(data, PACKET_LENGHT);
        debug("\nSbus (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(data, PACKET_LENGHT, "0x%X ");
        if (data[0] == 0x0F) {
            if (data[24] == 0x04 || data[24] == 0x14 || data[24] == 0x24 || data[24] == 0x34) {
                packet_id = data[24] >> 4;
                add_alarm_in_us(SLOT_0_DELAY - uart0_get_time_elapsed(), send_slot_callback, NULL, true);
                debug("\nSbus (%u) > ", uxTaskGetStackHighWaterMark(NULL));
            }
        }
    }
}

static int64_t send_slot_callback(alarm_id_t id, void *parameters) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint timestamp;
    static uint8_t index = 0;
    uint next_alarm;
    slot = index + packet_id * 8;
    if (context.debug == 2) {
        if (slot == 0 || slot == 8 || slot == 16 || slot == 24)
            printf("\nT:%u\n", uart0_get_time_elapsed());
        else
            printf("\nT:%u\n", time_us_32() - timestamp);
    }
    timestamp = time_us_32();
    if (index < 7) {
        index++;
        next_alarm = INTER_SLOT_DELAY - (time_us_32() - timestamp);
    } else {
        index = 0;
        next_alarm = 0;
    }
    vTaskResume(context.led_task_handle);
    slot_pending = true;
    vTaskNotifyGiveIndexedFromISR(context.uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return next_alarm;
}

static inline void send_slot(uint8_t slot) {
    debug(" (%u)", slot);
    uint16_t value = 0;
    if (sbus_sensor[slot]) {
        if (sbus_sensor[slot]->value) {
            value = format(sbus_sensor[slot]->data_id, *sbus_sensor[slot]->value);
        } else {
            value = format(sbus_sensor[slot]->data_id, 0);
        }
        uint8_t data[3];
        data[0] = get_slot_id(slot);
        data[1] = value;
        data[2] = value >> 8;
        uart0_write_bytes(data, 3);
        debug("%X:%X:%X ", data[0], data[1], data[2]);
    }
}

static uint16_t format(uint8_t data_id, float value) {
    static float coord = 0;
    if (data_id == SBUS_RPM) {
        return (uint16_t)round(value / 6);
    }
    if (data_id == SBUS_TEMP) {
        return (uint16_t)round(value + 100) | 0X8000;
    }
    if (data_id == SBUS_VOLT_V1) {
        return __builtin_bswap16((uint16_t)round(value * 10) | 0x8000);
    }
    if (data_id == SBUS_VOLT_V2) {
        return __builtin_bswap16((uint16_t)round(value * 10));
    }
    if (data_id == SBUS_VARIO_SPEED) {
        return __builtin_bswap16((int16_t)round(value * 100));
    }
    if (data_id == SBUS_VARIO_ALT) {
        return __builtin_bswap16((int16_t)round(value) | 0x4000);
    }
    if (data_id == SBUS_POWER_CURR) {
        return __builtin_bswap16((uint16_t)round(value * 100) | 0x4000);
    }
    if (data_id == SBUS_POWER_VOLT) {
        return __builtin_bswap16((uint16_t)round((value)*100));
    }
    if (data_id == SBUS_AIR_SPEED) {
        return __builtin_bswap16((uint16_t)round(value) | 0x4000);
    }
    if (data_id == SBUS_GPS_SPEED) {
        debug("\nGPS Speed: %f kmh\n", value);
        return __builtin_bswap16((uint16_t)round(value * 1.852) | 0x4000);
    }
    if (data_id == SBUS_GPS_VARIO_SPEED) {
        return __builtin_bswap16((int16_t)round(value * 10));
    }
    if (data_id == SBUS_GPS_ALTITUDE) {
        return __builtin_bswap16((int16_t)round(value) | 0x4000);
    }
    if (data_id == SBUS_GPS_LATITUDE1 || data_id == SBUS_GPS_LONGITUDE1) {
        // bits 1-4: bits 17-20 from minutes precision 4 (minutes*10000 = 20 bits)
        // bit 5: S/W bit
        // bits 9-16: degrees
        coord = value;
        uint16_t lat_lon = 0;
        if (coord < 0) {
            lat_lon = 1 << SBUS_SOUTH_WEST_BIT;
            coord *= -1;
        }
        uint8_t degrees = coord;
        lat_lon |= degrees << 8;
        uint32_t minutes = (coord - degrees) * 60 * 10000;  // minutes precision 4
        lat_lon |= (minutes >> 16) & 0xf;
        return __builtin_bswap16(lat_lon);
    }
    if (data_id == SBUS_GPS_LATITUDE2 || data_id == SBUS_GPS_LONGITUDE2) {
        // bits 1-16 from minutes precision 4 (minutes*10000 = 20 bits)
        uint32_t minutes = (coord - (int)coord) * 60 * 10000;  // minutes precision 4
        return __builtin_bswap16(minutes);
    }
    if (data_id == SBUS_GPS_TIME) {
        if (value > 120000) value -= 120000;
        uint8_t hours = value / 10000;
        uint8_t minutes = (uint8_t)(value / 100) - hours * 100;
        uint8_t seconds = value - hours * 10000 - minutes * 100;
        return __builtin_bswap16(hours * 3600 + minutes * 60 + seconds);
    }
    return __builtin_bswap16(round(value));
}

static uint8_t get_slot_id(uint8_t slot) {
    uint8_t slot_id[32] = {0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53,
                           0xD3, 0x33, 0xB3, 0x73, 0xF3, 0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB,
                           0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB};
    return slot_id[slot];
}

static void add_sensor(uint8_t slot, sensor_sbus_t *new_sensor) { sbus_sensor[slot] = new_sensor; }

static void set_config(void) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_sbus_t *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_fet};
        add_sensor(SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_bec};
        add_sensor(SLOT_TEMP2, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_fet};
        add_sensor(SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_bec};
        add_sensor(SLOT_TEMP2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage_bec};
        add_sensor(SLOT_POWER_VOLT3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current_bec};
        add_sensor(SLOT_POWER_CURR3, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage_bec};
        add_sensor(SLOT_POWER_VOLT3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current_bec};
        add_sensor(SLOT_POWER_CURR3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature};
        add_sensor(SLOT_TEMP1, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage_bec};
        add_sensor(SLOT_POWER_VOLT3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current_bec};
        add_sensor(SLOT_POWER_CURR3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_fet};
        add_sensor(SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_bec};
        add_sensor(SLOT_TEMP2, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature};
        add_sensor(SLOT_TEMP1, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature};
        add_sensor(SLOT_TEMP1, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_fet};
        add_sensor(SLOT_TEMP1, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temp_esc};
        add_sensor(SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temp_motor};
        add_sensor(SLOT_TEMP2, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temp_esc};
        add_sensor(SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temp_motor};
        add_sensor(SLOT_TEMP2, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_RPM, parameter.rpm};
        add_sensor(SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage};
        add_sensor(SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, parameter.voltage_bec};
        add_sensor(SLOT_POWER_VOLT3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current_bec};
        add_sensor(SLOT_POWER_CURR3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_fet};
        add_sensor(SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.temperature_bec};
        add_sensor(SLOT_TEMP2, new_sensor);
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_LATITUDE1, parameter.lat};
        add_sensor(SLOT_GPS_LAT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_LATITUDE2, parameter.lat};
        add_sensor(SLOT_GPS_LAT2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_LONGITUDE1, parameter.lon};
        add_sensor(SLOT_GPS_LON1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_LONGITUDE2, parameter.lon};
        add_sensor(SLOT_GPS_LON2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_ALTITUDE, parameter.alt};
        add_sensor(SLOT_GPS_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_SPEED, parameter.spd};
        add_sensor(SLOT_GPS_SPD, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_VARIO_SPEED, parameter.vspeed};
        add_sensor(SLOT_GPS_VARIO, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_GPS_TIME, parameter.time};
        add_sensor(SLOT_GPS_TIME, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        if (config->sbus_battery_slot) {
            new_sensor = malloc(sizeof(sensor_sbus_t));
            *new_sensor = (sensor_sbus_t){SBUS_VOLT_V1, parameter.voltage};
            add_sensor(SLOT_VOLT_V1, new_sensor);

            new_sensor = malloc(sizeof(sensor_sbus_t));
            *new_sensor = (sensor_sbus_t){SBUS_VOLT_V2, NULL};
            add_sensor(SLOT_VOLT_V2, new_sensor);
        } else {
            new_sensor = malloc(sizeof(sensor_sbus_t));
            *new_sensor = (sensor_sbus_t){SBUS_VOLT_V1, NULL};
            add_sensor(SLOT_VOLT_V1, new_sensor);

            new_sensor = malloc(sizeof(sensor_sbus_t));
            *new_sensor = (sensor_sbus_t){SBUS_VOLT_V2, parameter.voltage};
            add_sensor(SLOT_VOLT_V2, new_sensor);
        }
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CURR, parameter.current};
        add_sensor(SLOT_POWER_CURR2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_CONS, parameter.consumption};
        add_sensor(SLOT_POWER_CONS2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_POWER_VOLT, NULL};
        add_sensor(SLOT_POWER_VOLT2, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_TEMP, parameter.ntc};
        add_sensor(SLOT_TEMP1, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_VARIO_ALT, parameter.altitude};
        add_sensor(SLOT_VARIO_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_VARIO_SPEED, parameter.vspeed};
        add_sensor(SLOT_VARIO_SPEED, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_VARIO_ALT, parameter.altitude};
        add_sensor(SLOT_VARIO_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_VARIO_SPEED, parameter.vspeed};
        add_sensor(SLOT_VARIO_SPEED, new_sensor);
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
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_VARIO_ALT, parameter.altitude};
        add_sensor(SLOT_VARIO_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_VARIO_SPEED, parameter.vspeed};
        add_sensor(SLOT_VARIO_SPEED, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){SBUS_AIR_SPEED, parameter.airspeed};
        add_sensor(SLOT_AIR_SPEED, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
