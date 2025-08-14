#include "smartport.h"

#include <math.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

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
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gpio.h"
#include "gps.h"
#include "ms5611.h"
#include "ntc.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "stdlib.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define AIRCR_Register (*((volatile uint32_t *)(PPB_BASE + 0x0ED0C)))
// FrSky Smartport Data Id

#define UART
#define ALT_FIRST_ID 0x0100  // 100 m
#define ALT_LAST_ID 0x010f
#define VARIO_FIRST_ID 0x0110  // 100 m/s
#define VARIO_LAST_ID 0x011f
#define CURR_FIRST_ID 0x0200  // 10 A
#define CURR_LAST_ID 0x020f
#define VFAS_FIRST_ID 0x0210  // 100 v
#define VFAS_LAST_ID 0x021f
#define CELLS_FIRST_ID 0x0300  //
#define CELLS_LAST_ID 0x030f
#define T1_FIRST_ID 0x0400  // 1 C
#define T1_LAST_ID 0x040f
#define T2_FIRST_ID 0x0410  // 1 C
#define T2_LAST_ID 0x041f
#define RPM_FIRST_ID 0x0500  // 1 rpm
#define RPM_LAST_ID 0x050f
#define FUEL_FIRST_ID 0x0600  // 1 %
#define FUEL_LAST_ID 0x060f
#define ACCX_FIRST_ID 0x0700  // 100 g
#define ACCX_LAST_ID 0x070f
#define ACCY_FIRST_ID 0x0710  // 100 g
#define ACCY_LAST_ID 0x071f
#define ACCZ_FIRST_ID 0x0720  // 100 g
#define ACCZ_LAST_ID 0x072f
#define GPS_LONG_LATI_FIRST_ID 0x0800  // bit32(1<<31)=1=LON:=0=LAT, bit31(1<<30)=1=-:=0=+, escaler: 10000
#define GPS_LONG_LATI_LAST_ID 0x080f
#define GPS_ALT_FIRST_ID 0x0820  // 100 m
#define GPS_ALT_LAST_ID 0x082f
#define GPS_SPEED_FIRST_ID 0x0830  // 1000 kts
#define GPS_SPEED_LAST_ID 0x083f
#define GPS_COURS_FIRST_ID 0x0840  // 100 º
#define GPS_COURS_LAST_ID 0x084f
#define GPS_TIME_DATE_FIRST_ID 0x0850  // Date: Y M D 0xFF or Time: H M S 0x00
#define GPS_TIME_DATE_LAST_ID 0x085f
#define A3_FIRST_ID 0x0900  // 100 v
#define A3_LAST_ID 0x090f
#define A4_FIRST_ID 0x0910  // 100 v
#define A4_LAST_ID 0x091f
#define AIR_SPEED_FIRST_ID 0x0a00  // 10 kts
#define AIR_SPEED_LAST_ID 0x0a0f
#define RBOX_BATT1_FIRST_ID 0x0b00  // 1000 v, 100 A
#define RBOX_BATT1_LAST_ID 0x0b0f
#define RBOX_BATT2_FIRST_ID 0x0b10  // 1000 v, 100 A
#define RBOX_BATT2_LAST_ID 0x0b1f
#define RBOX_STATE_FIRST_ID 0x0b20  // 1
#define RBOX_STATE_LAST_ID 0x0b2f
#define RBOX_CNSP_FIRST_ID 0x0b30  // 1 mAh (1), 1mAh (2)
#define RBOX_CNSP_LAST_ID 0x0b3f
#define SD1_FIRST_ID 0x0b40
#define SD1_LAST_ID 0x0b4f
#define ESC_POWER_FIRST_ID 0x0b50  // bytes 1,2: 100 V,  bytes 3,4: 100 A
#define ESC_POWER_LAST_ID 0x0b5f
#define ESC_RPM_CONS_FIRST_ID 0x0b60  // bytes 1,2: 0.01 rpm,  bytes 3,4: 1 mah
#define ESC_RPM_CONS_LAST_ID 0x0b6f
#define ESC_TEMPERATURE_FIRST_ID 0x0b70  // 1 C
#define ESC_TEMPERATURE_LAST_ID 0x0b7f
#define X8R_FIRST_ID 0x0c20
#define X8R_LAST_ID 0x0c2f
#define S6R_FIRST_ID 0x0c30
#define S6R_LAST_ID 0x0c3f
#define GASSUIT_TEMP1_FIRST_ID 0x0d00  // 1 C
#define GASSUIT_TEMP1_LAST_ID 0x0d0f
#define GASSUIT_TEMP2_FIRST_ID 0x0d10  // 1 C
#define GASSUIT_TEMP2_LAST_ID 0x0d1f
#define GASSUIT_SPEED_FIRST_ID 0x0d20  // 1 rpm
#define GASSUIT_SPEED_LAST_ID 0x0d2f
#define GASSUIT_RES_VOL_FIRST_ID 0x0d30  // 1 ml
#define GASSUIT_RES_VOL_LAST_ID 0x0d3f
#define GASSUIT_RES_PERC_FIRST_ID 0x0d40  // 1 %
#define GASSUIT_RES_PERC_LAST_ID 0x0d4f
#define GASSUIT_FLOW_FIRST_ID 0x0d50  // 1 ml/min
#define GASSUIT_FLOW_LAST_ID 0x0d5f
#define GASSUIT_MAX_FLOW_FIRST_ID 0x0d60  // 1 ml/min
#define GASSUIT_MAX_FLOW_LAST_ID 0x0d6f
#define GASSUIT_AVG_FLOW_FIRST_ID 0x0d70  // 1 ml/min
#define GASSUIT_AVG_FLOW_LAST_ID 0x0d7f
#define SBEC_POWER_FIRST_ID 0x0e50  // bytes 1,2: 100 V,  bytes 3,4: 100 A
#define SBEC_POWER_LAST_ID 0x0e5f
#define DIY_FIRST_ID 0x5100
#define DIY_LAST_ID 0x52ff
#define DIY_STREAM_FIRST_ID 0x5000
#define DIY_STREAM_LAST_ID 0x50ff
#define FACT_TEST_ID 0xf000
#define RSSI_ID 0xf101
#define A1_ID 0xf102  // 10 v
#define A2_ID 0xf103  // 10 v
#define SP2UART_A_ID 0xfd00
#define SP2UART_B_ID 0xfd01
#define RXBT_ID 0xf104  // 10 v
#define RAS_ID 0xf105
#define XJT_VERSION_ID 0xf106
#define FUEL_QTY_FIRST_ID 0x0a10  // 100 ml
#define FUEL_QTY_LAST_ID 0x0a1f

#define TIMEOUT_US 500
#define PACKET_LENGHT 2

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

typedef struct smartport_sensor_cell_individual_parameters_t {
    uint8_t *cell_count;
    float *cell_voltage[18];
    uint16_t rate;
} smartport_sensor_cell_individual_parameters_t;

typedef struct smartport_packet_parameters_t {
    uint16_t data_id;
    QueueHandle_t queue_handle;
} smartport_packet_parameters_t;

typedef struct smartport_packet_t {
    uint16_t frame_id;
    uint16_t data_id;
    uint32_t value;
} smartport_packet_t;

static SemaphoreHandle_t semaphore_sensor = NULL;
static bool is_maintenance_mode = false;
static const uint8_t sensor_id_matrix[29] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9,
                                             0x6A, 0xCB, 0xAC, 0xD,  0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53,
                                             0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B, 0x0};
static TaskHandle_t packet_task_handle;
static QueueHandle_t packet_queue_handle;
config_t *config_lua;

static void sensor_task(void *parameters);
static void sensor_void_task(void *parameters);
static void sensor_double_task(void *parameters);
static void sensor_coordinates_task(void *parameters);
static void sensor_datetime_task(void *parameters);
static void sensor_cell_task(void *parameters);
static void packet_task(void *parameters);
static void process(smartport_parameters_t *parameter);
static void process_packet(smartport_parameters_t *parameter, uint8_t frame_id, uint16_t data_id, uint32_t value);
static int32_t format(uint16_t data_id, float value);
static uint32_t format_double(uint16_t data_id, float value_l, float value_h);
static uint32_t format_coordinate(coordinate_type_t type, float value);
static uint32_t format_datetime(uint8_t type, uint32_t value);
static uint32_t format_cell(uint8_t cell_index, float value);
static void send_packet(uint8_t frame_id, uint16_t data_id, uint32_t value);
static void send_byte(uint8_t c, uint16_t *crcp);
static void set_config(smartport_parameters_t *parameter);
static uint8_t sensor_id_to_crc(uint8_t sensor_id);
static uint8_t sensor_crc_to_id(uint8_t sensor_id_crc);
static uint8_t get_crc(uint8_t *data);
static int64_t reboot_callback(alarm_id_t id, void *user_data);

void smartport_task(void *parameters) {
    smartport_parameters_t parameter;
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(57600, UART_RECEIVER_TX, UART_RECEIVER_RX, TIMEOUT_US, 8, 1, UART_PARITY_NONE, true, true);
    semaphore_sensor = xSemaphoreCreateBinary();
    xSemaphoreTake(semaphore_sensor, 0);
    set_config(&parameter);
    packet_queue_handle = xQueueCreate(32, sizeof(smartport_packet_t));
    xTaskCreate(packet_task, "packet_task", STACK_SMARTPORT_PACKET_TASK, (void *)&parameter.data_id, 3,
                &packet_task_handle);
    xQueueSendToBack(context.tasks_queue_handle, packet_task_handle, 0);
    // TaskHandle_t task_handle;
    // xTaskCreate(sensor_void_task, "sensor_void_task", STACK_SMARTPORT_SENSOR_VOID_TASK, NULL, 2, &task_handle);
    // xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
    debug("\nSmartport init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(smartport_parameters_t *parameter) {
    uint lenght = uart0_available();
    if (lenght) {
        uint8_t data[lenght];
        if (context.debug == 2 && lenght != 2) {
            printf("\n");
            debug_buffer2(data, lenght, "0x%X ");
        }
        uart0_read_bytes(data, lenght);
        if (data[0] == 0x7E && data[1] == sensor_id_to_crc(parameter->sensor_id)) {
            if (lenght == PACKET_LENGHT) {
                debug("\nSmartport (%u) < ", uxTaskGetStackHighWaterMark(NULL));
                debug_buffer(data, PACKET_LENGHT, "0x%X ");
                if (/*is_maintenance_mode &&*/ uxQueueMessagesWaiting(packet_queue_handle)) {
                    xTaskNotifyGive(packet_task_handle);
                } else if (!is_maintenance_mode) {
                    xSemaphoreGive(semaphore_sensor);
                    vTaskDelay(4 / portTICK_PERIOD_MS);
                    xSemaphoreTake(semaphore_sensor, 0);
                }
            } else if (lenght >= 10) {
                uint8_t i;
                uint8_t delta = 0;
                for (i = 0; i < lenght; i++) {
                    data[i] = data[i + delta];
                    if (data[i] == 0x7D) {
                        delta++;
                        data[i] = data[i + delta] ^ 0x20;
                    }
                }
                if (i == 10) {
                    uint8_t crc = get_crc(data);
                    if (crc == data[9]) {
                        uint8_t frame_id = data[2];
                        uint16_t data_id = (uint16_t)data[4] << 8 | data[3];
                        uint value =
                            (uint32_t)data[8] << 24 | (uint32_t)data[7] << 16 | (uint16_t)data[6] << 8 | data[5];
                        debug("\nSmartport. Received packet (%u) FrameId 0x%X DataId 0x%X Value 0x%X < ",
                              uxTaskGetStackHighWaterMark(NULL), frame_id, data_id, value);
                        debug_buffer(data, 10, "0x%X ");
                        process_packet(parameter, frame_id, data_id, value);
                    }
                }
            }
        }
    }
}

static void process_packet(smartport_parameters_t *parameter, uint8_t frame_id, uint16_t data_id, uint32_t value) {
    // set maintenance mode on
    if (frame_id == 0x21 && data_id == 0xFFFF && value == 0x80) {
        is_maintenance_mode = true;
        debug("\nSmartport (%u). Maintenance mode ON ", uxTaskGetStackHighWaterMark(NULL));
        config_t *config = config_read();
        debug("\nDataId 0x%0X 0x%X", parameter->data_id, config->smartport_data_id);
        return;
    }

    // set maintenance mode off
    if (frame_id == 0x20 && data_id == 0xFFFF && value == 0x80) {
        is_maintenance_mode = false;
        debug("\nSmartport (%u). Maintenance mode OFF ", uxTaskGetStackHighWaterMark(NULL));
        return;
    }

    // send sensor id
    if (is_maintenance_mode && frame_id == 0x30 && data_id == parameter->data_id && value == 0x01) {
        smartport_packet_t packet;
        packet.value = (parameter->sensor_id - 1) << 8 | 0x01;
        packet.frame_id = 0x32;
        packet.data_id = parameter->data_id;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        debug("\nSmartport (%u). Send sensor Id %i (0x%X)", uxTaskGetStackHighWaterMark(NULL), parameter->sensor_id,
              sensor_id_to_crc(parameter->sensor_id));
        return;
    }

    // change sensor id
    if (is_maintenance_mode && frame_id == 0x31 && data_id == parameter->data_id && (uint8_t)value == 0x01) {
        uint8_t sensor_id = (value >> 8) + 1;
        config_t *config = config_read();
        config->smartport_sensor_id = sensor_id;
        config_write(config);
        debug("\nSmartport (%u). Change sensor Id %i (0x%X)", uxTaskGetStackHighWaterMark(NULL), sensor_id,
              sensor_id_to_crc(sensor_id));
        return;
    }

    // send config (old)
    if (is_maintenance_mode && frame_id == 0x30 && data_id == parameter->data_id && value == 0) {
        uint32_t value = 0;
        smartport_packet_t packet;
        config_t *config = config_read();
        // packet 1
        value = 0xF1;
        // value |= (uint32_t)VERSION_PATCH << 8;
        // value |= (uint32_t)VERSION_MINOR << 16;
        // value |= (uint32_t)VERSION_MAJOR << 24;
        packet.frame_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.value = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        // packet 2
        value = 0xF2;
        value |= config->enable_analog_airspeed << 8;
        value |= config->enable_gps << 9;
        value |= config->enable_analog_voltage << 10;
        value |= config->enable_analog_current << 12;
        value |= config->enable_analog_ntc << 13;
        value |= config->enable_pwm_out << 15;
        value |= ((uint8_t)(config->refresh_rate_rpm / 100) & 0b1111) << 16;
        value |= ((uint8_t)(config->refresh_rate_voltage / 100) & 0b1111) << 20;
        value |= ((uint8_t)(config->refresh_rate_current / 100) & 0b1111) << 24;
        value |= ((uint8_t)(config->refresh_rate_temperature / 100) & 0b1111) << 28;
        packet.frame_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.value = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        // packet 3
        value = 0xF3;
        value |= (ELEMENTS(config->alpha_rpm) & 0b1111) << 8;
        value |= (ELEMENTS(config->alpha_voltage) & 0b1111) << 12;
        value |= (ELEMENTS(config->alpha_current) & 0b1111) << 16;
        value |= (ELEMENTS(config->alpha_temperature) & 0b1111) << 20;
        value |= config->esc_protocol << 24;
        packet.frame_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.value = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        // packet 4
        value = 0xF4;
        value |= config->i2c_module << 8;
        value |= config->i2c_address << 12;
        packet.frame_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.value = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        debug("\nSmartport (%u). Send config...", uxTaskGetStackHighWaterMark(NULL));
        return;
    }

    // sensor id search
    if (frame_id == 0x35 && data_id == 0x5200) {
        smartport_packet_t packet;
        packet.value = 1;
        packet.frame_id = 0x32;
        packet.data_id = 0x5201;
        is_maintenance_mode = true;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
    }

    // send config
    if (frame_id == 0x34) {
        config_t *config = config_read();
        uint8_t frame_id_send = 0x32;
        bool send = true;
        smartport_packet_t packet;
        packet.frame_id = 0x32;
        packet.data_id = data_id;
        switch (data_id) {
            case 0x5101: {
                packet.value = 0;
                char version[] = PROJECT_VERSION;
                memmove(version, version + 1, strlen(version));
                char *token;
                token = strtok(version, " . ");
                while (token != NULL) {
                    packet.value = packet.value << 8 | atoi(token);
                    token = strtok(NULL, " . ");
                }
                break;
            }
            case 0x5102:
                packet.value = config->rx_protocol;
                break;
            case 0x5103:
                packet.value = config->esc_protocol;
                break;
            case 0x5104:
                packet.value = config->enable_gps;
                break;
            case 0x5105:
                packet.value = config->gps_baudrate;
                break;
            case 0x5106:
                packet.value = config->enable_analog_voltage;
                break;
            case 0x5107:
                packet.value = config->enable_analog_current;
                break;
            case 0x5108:
                packet.value = config->enable_analog_ntc;
                break;
            case 0x5109:
                packet.value = config->enable_analog_airspeed;
                break;
            case 0x510A:
                packet.value = config->i2c_module;
                break;
            case 0x510B:
                packet.value = config->i2c_address;
                break;
            case 0x510C:
                packet.value = ELEMENTS(config->alpha_rpm);
                break;
            case 0x510D:
                packet.value = ELEMENTS(config->alpha_voltage);
                break;
            case 0x510E:
                packet.value = ELEMENTS(config->alpha_current);
                break;
            case 0x510F:
                packet.value = ELEMENTS(config->alpha_temperature);
                break;
            case 0x5110:
                packet.value = ELEMENTS(config->alpha_vario);
                break;
            case 0x5111:
                packet.value = ELEMENTS(config->alpha_airspeed);
                break;
            case 0x5112:
                packet.value = config->refresh_rate_rpm;
                break;
            case 0x5113:
                packet.value = config->refresh_rate_voltage;
                break;
            case 0x5114:
                packet.value = config->refresh_rate_current;
                break;
            case 0x5115:
                packet.value = config->refresh_rate_temperature;
                break;
            case 0x5116:
                packet.value = config->refresh_rate_gps;
                break;
            case 0x5117:
                packet.value = config->refresh_rate_consumption;
                break;
            case 0x5118:
                packet.value = config->refresh_rate_vario;
                break;
            case 0x5119:
                packet.value = config->refresh_rate_airspeed;
                break;
            case 0x511A:
                packet.value = config->refresh_rate_default;
                break;
            case 0x511B:
                packet.value = config->analog_voltage_multiplier * 100;
                break;
            case 0x511C:
                packet.value = config->analog_current_type;
                break;
            case 0x511D:
                packet.value = config->gpio_interval;
                break;
            case 0x511E:
                packet.value = config->analog_current_quiescent_voltage;
                break;
            case 0x511F:
                packet.value = config->analog_current_multiplier;
                break;
            case 0x5120:
                packet.value = config->analog_current_offset * 100;
                break;
            case 0x5121:
                packet.value = config->analog_current_autoffset;
                break;
            case 0x5122:
                packet.value = config->pairOfPoles;
                break;
            case 0x5123:
                packet.value = config->mainTeeth;
                break;
            case 0x5124:
                packet.value = config->pinionTeeth;
                break;
            case 0x5125:
                packet.value = config->rpm_multiplier;
                break;
            case 0x5126:
                packet.value = config->bmp280_filter;
                break;
            case 0x5127:
                packet.value = config->enable_pwm_out;
                break;
            case 0x5128:
                packet.value = config->smartport_sensor_id;
                break;
            case 0x5129:
                packet.value = config->smartport_data_id;
                break;
            case 0x512A:
                packet.value = config->vario_auto_offset;
                break;
            case 0x512B:
                packet.value = config->xbus_clock_stretch;
                break;
            case 0x512C:
                packet.value = config->jeti_gps_speed_units_kmh;
                break;
            case 0x512D:
                packet.value = config->enable_esc_hw4_init_delay;
                break;
            case 0x512E:
                packet.value = config->esc_hw4_init_delay_duration;
                break;
            case 0x512F:
                packet.value = config->esc_hw4_current_thresold;
                break;
            case 0x5130:
                packet.value = config->esc_hw4_current_max;
                break;
            case 0x5131:
                packet.value = config->esc_hw4_divisor * 100;
                break;
            case 0x5132:
                packet.value = config->esc_hw4_current_multiplier * 100;
                break;
            case 0x5133:
                packet.value = config->ibus_alternative_coordinates;
                break;
            case 0x5134:
                packet.value = config->debug;
                break;
            case 0x5135:
                packet.value = config->esc_hw4_is_manual_offset;
                break;
            case 0x5136:
                packet.value = config->analog_rate;
                break;
            case 0x5137:
                packet.value = config->xbus_use_alternative_volt_temp;
                break;
            case 0x5138:
                packet.value = config->gpio_mask;
                break;
            case 0x5139:
                packet.value = config->esc_hw4_offset;
                break;
            case 0x513A:
                packet.value = config->serial_monitor_baudrate;
                break;
            case 0x513B:
                packet.value = config->serial_monitor_stop_bits;
                break;
            case 0x513C:
                packet.value = config->serial_monitor_parity;
                break;
            case 0x513D:
                packet.value = config->serial_monitor_timeout_ms;
                break;
            case 0x513E:
                packet.value = config->serial_monitor_inverted;
                break;
            case 0x513F:
                packet.value = config->airspeed_offset * 100;
                break;
            case 0x5140:
                packet.value = config->airspeed_slope * 100;
                break;
            case 0x5141:
                packet.value = config->fuel_flow_ml_per_pulse * 10000;
                break;
            case 0x5142:
                packet.value = config->enable_fuel_flow;
                break;
            case 0x5143:
                packet.value = config->xgzp68xxd_k;
                break;
            case 0x5144:
                packet.value = config->enable_fuel_pressure;
                break;
            case 0x5145:
                packet.value = config->smart_esc_calc_consumption;
                break;
            case 0x5146:
                packet.value = config->serial_monitor_gpio;
                break;
            case 0x5147:
                packet.value = config->gps_rate;
                break;
            case 0x5148:
                packet.value = config->serial_monitor_format;
                break;
            case 0x5149:
                packet.value = config->gps_protocol;
                break;
            case 0x514A:
                packet.value = config->sbus_battery_slot;
                break;
            default:
                send = false;
                debug("\nSmartport. Unknown request frameId 0x%X dataId 0x%X", frame_id, data_id);
                break;
        }
        if (send) xQueueSendToBack(packet_queue_handle, &packet, 0);
    }

    // receive config
    if (frame_id == 0x35 && data_id == 0x5201 && value == 0) {
        config_lua = malloc(sizeof(config_t));
        memcpy(config_lua, config_read(), sizeof(config_t));
        debug("\nSmartport. Start saving...");
    }
    if (frame_id == 0x35 && data_id == 0x5201 && value == 1) {
        config_write(config_lua);
        is_maintenance_mode = false;
        free(config_lua);
        debug("\nSmartport. Complete save config");
    }
    if (frame_id == 0x33) {
        uint8_t frame_id_send = 0x32;
        bool write = true;
        switch (data_id) {
            case 0x5102:
                config_lua->rx_protocol = value;
                break;
            case 0x5103:
                config_lua->esc_protocol = value;
                break;
            case 0x5104:
                config_lua->enable_gps = value;
                break;
            case 0x5105:
                config_lua->gps_baudrate = value;
                break;
            case 0x5106:
                config_lua->enable_analog_voltage = value;
                break;
            case 0x5107:
                config_lua->enable_analog_current = value;
                break;
            case 0x5108:
                config_lua->enable_analog_ntc = value;
                break;
            case 0x5109:
                config_lua->enable_analog_airspeed = value;
                break;
            case 0x510A:
                config_lua->i2c_module = value;
                break;
            case 0x510B:
                config_lua->i2c_address = value;
                break;
            case 0x510C:
                config_lua->alpha_rpm = value;
                break;
            case 0x510D:
                config_lua->alpha_voltage = value;
                break;
            case 0x510E:
                config_lua->alpha_current = value;
                break;
            case 0x510F:
                config_lua->alpha_temperature = value;
                break;
            case 0x5110:
                config_lua->alpha_vario = value;
                break;
            case 0x5111:
                config_lua->alpha_airspeed = value;
                break;
            case 0x5112:
                config_lua->refresh_rate_rpm = value;
                debug("\nRPM: %u", config_lua->refresh_rate_rpm);
                break;
            case 0x5113:
                config_lua->refresh_rate_voltage = value;
                debug("\nRPM: %u", config_lua->refresh_rate_rpm);
                break;
            case 0x5114:
                config_lua->refresh_rate_current = value;
                break;
            case 0x5115:
                config_lua->refresh_rate_temperature = value;
                break;
            case 0x5116:
                config_lua->refresh_rate_gps = value;
                break;
            case 0x5117:
                config_lua->refresh_rate_consumption = value;
                break;
            case 0x5118:
                config_lua->refresh_rate_vario = value;
                break;
            case 0x5119:
                config_lua->refresh_rate_airspeed = value;
                break;
            case 0x511A:
                config_lua->refresh_rate_default = value;
                break;
            case 0x511B:
                config_lua->analog_voltage_multiplier = value / 100;
                break;
            case 0x511C:
                config_lua->analog_current_type = value;
                break;
            case 0x511D:
                config_lua->gpio_interval = value;
                break;
            case 0x511E:
                config_lua->analog_current_quiescent_voltage = value;
                break;
            case 0x511F:
                config_lua->analog_current_multiplier = value;
                break;
            case 0x5120:
                config_lua->analog_current_offset = value / 100;
                break;
            case 0x5121:
                config_lua->analog_current_autoffset = value;
                break;
            case 0x5122:
                config_lua->pairOfPoles = value;
                break;
            case 0x5123:
                config_lua->mainTeeth = value;
                break;
            case 0x5124:
                config_lua->pinionTeeth = value;
                break;
            case 0x5125:
                config_lua->rpm_multiplier = value;
                break;
            case 0x5126:
                config_lua->bmp280_filter = value;
                break;
            case 0x5127:
                config_lua->enable_pwm_out = value;
                break;
            case 0x5128:
                config_lua->smartport_sensor_id = value;
                break;
            case 0x5129:
                config_lua->smartport_data_id = value;
                break;
            case 0x512A:
                config_lua->vario_auto_offset = value;
                break;
            case 0x512B:
                config_lua->xbus_clock_stretch = value;
                break;
            case 0x512C:
                config_lua->jeti_gps_speed_units_kmh = value;
                break;
            case 0x512D:
                config_lua->enable_esc_hw4_init_delay = value;
                break;
            case 0x512E:
                config_lua->esc_hw4_init_delay_duration = value;
                break;
            case 0x512F:
                config_lua->esc_hw4_current_thresold = value;
                break;
            case 0x5130:
                config_lua->esc_hw4_current_max = value;
                break;
            case 0x5131:
                config_lua->esc_hw4_divisor = value / 100;
                break;
            case 0x5132:
                config_lua->esc_hw4_current_multiplier = value / 100;
                break;
            case 0x5133:
                config_lua->ibus_alternative_coordinates = value;
                break;
            case 0x5134:
                config_lua->debug = value;
                break;
            case 0x5135:
                config_lua->esc_hw4_is_manual_offset = value;
                break;
            case 0x5136:
                config_lua->analog_rate = value;
                break;
            case 0x5137:
                config_lua->xbus_use_alternative_volt_temp = value;
                break;
            case 0x5138:
                config_lua->gpio_mask = value;
                break;
            case 0x5139:
                config_lua->esc_hw4_offset = value;
                break;
            case 0x513A:
                config_lua->serial_monitor_baudrate = value;
                break;
            case 0x513B:
                config_lua->serial_monitor_stop_bits = value;
                break;
            case 0x513C:
                config_lua->serial_monitor_parity = value;
                break;
            case 0x513D:
                config_lua->serial_monitor_timeout_ms = value;
                break;
            case 0x513E:
                config_lua->serial_monitor_inverted = value;
                break;
            case 0x513F:
                config_lua->airspeed_offset = value / 100;
                break;
            case 0x5140:
                config_lua->airspeed_slope = value / 100;
                break;
            case 0x5141:
                config_lua->fuel_flow_ml_per_pulse = value / 10000;
                break;
            case 0x5142:
                config_lua->enable_fuel_flow = value;
                break;
            case 0x5143:
                config_lua->xgzp68xxd_k = value;
                break;
            case 0x5144:
                config_lua->enable_fuel_pressure = value;
                break;
            case 0x5145:
                config_lua->smart_esc_calc_consumption = value;
                break;
            case 0x5146:
                config_lua->serial_monitor_gpio = value;
                break;
            case 0x5147:
                config_lua->gps_rate = value;
                break;
            case 0x5148:
                config_lua->serial_monitor_format = value;
                break;
            case 0x5149:
                config_lua->gps_protocol = value;
                break;
            case 0x514A:
                config_lua->sbus_battery_slot = value;
                break;
            default:
                debug("\nSmartport. Unknown save request. frameId 0x%X dataId 0x%X", frame_id, data_id);
                break;
        }
        debug("\nSmartport. Store config. frameId 0x%X dataId 0x%X value %u", frame_id, data_id, value);
    }
}

static int64_t reboot_callback(alarm_id_t id, void *user_data) { AIRCR_Register = 0x5FA0004; }

static void sensor_task(void *parameters) {
    smartport_sensor_parameters_t parameter = *(smartport_sensor_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        int32_t data_formatted = format(parameter.data_id, *parameter.value);
        debug("\nSmartport. Sensor (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, parameter.data_id, data_formatted);
    }
}

static void sensor_gpio_task(void *parameters) {
    smartport_sensor_gpio_parameters_t parameter = *(smartport_sensor_gpio_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    uint cont = 0;
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (parameter.gpio_mask) {
            while (!(parameter.gpio_mask & (1 << cont))) {
                cont++;
                if (cont == 6) cont = 0;
            }
            float value = *parameter.value & (1 << cont) ? 1 : 0;
            uint16_t data_id = parameter.data_id + 17 + cont;
            int32_t data_formatted = format(data_id, value);
            debug("\nSmartport. Sensor GPIO (%u) > GPIO: %u STATE: %u > ", uxTaskGetStackHighWaterMark(NULL), 17 + cont,
                  (uint)value);
            send_packet(0x10, data_id, data_formatted);
            cont++;
            if (cont == 6) cont = 0;
        }
    }
}

static void sensor_void_task(void *parameters) {
    while (1) {
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (context.debug == 2) printf("\nSmartport. Sensor void (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0, 0, 0);
    }
}

static void sensor_double_task(void *parameters) {
    smartport_sensor_double_parameters_t parameter = *(smartport_sensor_double_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted = format_double(parameter.data_id, *parameter.value_l, *parameter.value_h);
        debug("\nSmartport. Sensor double (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, parameter.data_id, data_formatted);
    }
}

static void sensor_coordinates_task(void *parameters) {
    smartport_sensor_coordinate_parameters_t parameter = *(smartport_sensor_coordinate_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted;
        if (parameter.type == SMARTPORT_LATITUDE)
            data_formatted = format_coordinate(parameter.type, *parameter.latitude);
        else
            data_formatted = format_coordinate(parameter.type, *parameter.longitude);
        parameter.type = !parameter.type;
        debug("\nSmartport. Sensor coordinates (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, GPS_LONG_LATI_FIRST_ID, data_formatted);
    }
}

static void sensor_datetime_task(void *parameters) {
    smartport_sensor_datetime_parameters_t parameter = *(smartport_sensor_datetime_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted;
        if (parameter.type == SMARTPORT_DATE)
            data_formatted = format_datetime(parameter.type, *parameter.date);
        else
            data_formatted = format_datetime(parameter.type, *parameter.time);
        parameter.type = !parameter.type;
        debug("\nSmartport. Sensor datetime (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, GPS_TIME_DATE_FIRST_ID, data_formatted);
    }
}

static void sensor_cell_task(void *parameters) {
    smartport_sensor_cell_parameters_t parameter = *(smartport_sensor_cell_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    uint8_t cell_index = 0;
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (!*parameter.cell_count) return;
        uint32_t data_formatted = format_cell(cell_index, *parameter.cell_voltage);
        cell_index++;
        if (cell_index > *parameter.cell_count - 1) cell_index = 0;
        debug("\nSmartport. Sensor cell (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, CELLS_FIRST_ID, data_formatted);
    }
}

static void sensor_cell_individual_task(void *parameters) {
    smartport_sensor_cell_individual_parameters_t parameter =
        *(smartport_sensor_cell_individual_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    uint8_t cell_index = 0;
    while (1) {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (!*parameter.cell_count) return;
        uint32_t data_formatted = format_cell(cell_index, *parameter.cell_voltage[cell_index]);
        cell_index++;
        if (cell_index > *parameter.cell_count - 1) cell_index = 0;
        debug("\nSmartport. Sensor cell (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, CELLS_FIRST_ID, data_formatted);
    }
}

static void packet_task(void *parameters) {
    uint16_t data_id = *(uint16_t *)parameters;
    smartport_packet_t packet;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xQueueReceive(packet_queue_handle, &packet, 0);
        debug("\nSmartport. Packet (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(packet.frame_id, packet.data_id, packet.value);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}

static int32_t format(uint16_t data_id, float value) {
    if ((data_id >= GPS_SPEED_FIRST_ID && data_id <= GPS_SPEED_LAST_ID) ||
        (data_id >= RBOX_BATT1_FIRST_ID && data_id <= RBOX_BATT2_FIRST_ID))
        return round(value * 1000);

    if ((data_id >= ALT_FIRST_ID && data_id <= VARIO_LAST_ID) ||
        (data_id >= VFAS_FIRST_ID && data_id <= VFAS_LAST_ID) ||
        (data_id >= ACCX_FIRST_ID && data_id <= GPS_ALT_LAST_ID) ||
        (data_id >= GPS_COURS_FIRST_ID && data_id <= GPS_COURS_LAST_ID) ||
        (data_id >= A3_FIRST_ID && data_id <= A4_LAST_ID) || data_id == DIY_FIRST_ID + 5)
        return round(value * 100);

    if ((data_id >= CURR_FIRST_ID && data_id <= CURR_LAST_ID) ||
        (data_id >= AIR_SPEED_FIRST_ID && data_id <= AIR_SPEED_LAST_ID) || data_id == A1_ID || data_id == A2_ID ||
        data_id == RXBT_ID)
        return round(value * 10);

    return round(value);
}

static uint32_t format_double(uint16_t data_id, float value_l, float value_h) {
    if ((data_id >= ESC_POWER_FIRST_ID && data_id <= ESC_POWER_LAST_ID) ||
        (data_id >= SBEC_POWER_FIRST_ID && data_id <= SBEC_POWER_LAST_ID))
        return (uint32_t)round(value_h * 100) << 16 | (uint16_t)round(value_l * 100);

    if (data_id >= ESC_RPM_CONS_FIRST_ID && data_id <= ESC_RPM_CONS_LAST_ID) {
        return (uint32_t)round(value_h) << 16 | (uint16_t)round((value_l) / 100);
    }

    return (uint16_t)round(value_h * 500) << 8 | (uint16_t)value_l;
}

static uint32_t format_coordinate(coordinate_type_t type, float value) {
    uint32_t data = 0;
    if (value < 0) {
        data |= (uint32_t)1 << 30;
        value *= -1;
    }
    if (type == SMARTPORT_LONGITUDE) {
        data |= (uint32_t)1 << 31;
    }
    data |= (uint32_t)(value * 60 * 10000);  // deg to min * 10000
    return data;
}

static uint32_t format_datetime(uint8_t type, uint32_t value) {
    uint8_t dayHour = value / 10000;
    uint8_t monthMin = value / 100 - dayHour * 100;
    uint8_t yearSec = value - (value / 100) * 100;
    if (type == SMARTPORT_DATE) {
        return (uint32_t)yearSec << 24 | (uint32_t)monthMin << 16 | dayHour << 8 | 0xFF;
    }
    return (uint32_t)dayHour << 24 | (uint32_t)monthMin << 16 | yearSec << 8;
}

static uint32_t format_cell(uint8_t cell_index, float value) { return cell_index | (uint16_t)round(value * 500) << 8; }

static void send_byte(uint8_t c, uint16_t *crcp) {
    if (crcp != NULL) {
        uint16_t crc = *crcp;
        crc += c;
        crc += crc >> 8;
        crc &= 0x00FF;
        *crcp = crc;
    }
    if (c == 0x7D || c == 0x7E) {
        uart0_write(c);
        c ^= 0x20;
    }
    uart0_write(c);
    debug("%X ", c);
}

static void send_packet(uint8_t frame_id, uint16_t data_id, uint32_t value) {
    uint16_t crc = 0;
    uint8_t *u8p;
    // frame_id
    send_byte(frame_id, &crc);
    // data_id
    u8p = (uint8_t *)&data_id;
    send_byte(u8p[0], &crc);
    send_byte(u8p[1], &crc);
    // value
    u8p = (uint8_t *)&value;
    send_byte(u8p[0], &crc);
    send_byte(u8p[1], &crc);
    send_byte(u8p[2], &crc);
    send_byte(u8p[3], &crc);
    // crc
    send_byte(0xFF - (uint8_t)crc, NULL);
    // blink
    vTaskResume(context.led_task_handle);
}

static void set_config(smartport_parameters_t *parameter) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *baro_temp = NULL, *baro_pressure = NULL;
    parameter->sensor_id = config->smartport_sensor_id;
    parameter->data_id = 0x5000;  // config->smartport_data_id;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = NULL;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = NULL;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
                                          config->esc_hw4_divisor,
                                          config->esc_hw4_current_multiplier,
                                          config->esc_hw4_current_thresold,
                                          config->esc_hw4_current_max,
                                          config->esc_hw4_is_manual_offset,
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

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temperature_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID + 1;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID + 1;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_SMART) {
        smart_esc_parameters_t parameter;
        parameter.calc_consumption = config->smart_esc_calc_consumption;
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
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_individual_parameters_t parameter_sensor_cell;
        // rpm & consumption
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // voltage & current
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // bec. voltage & current
        parameter_sensor_double.data_id = SBEC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // temp_fet
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // temp_bec
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // temp_bat
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temperature_bat;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // current_bat
        parameter_sensor.data_id = CURR_FIRST_ID + 1;
        parameter_sensor.value = parameter.current_bat;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // cells
        parameter_sensor_cell.cell_count = parameter.cells;
        for (uint i = 0; i < 18; i++) parameter_sensor_cell.cell_voltage[i] = parameter.cell[i];
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // cycles
        /*parameter_sensor.data_id = DIY_FIRST_ID + 100;
        parameter_sensor.value = parameter.cycles;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_cell_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);*/
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temp_esc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temp_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE,
                    (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temp_esc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 2;
        parameter_sensor.value = parameter.temp_motor;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell,
                    3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_coordinate_parameters_t parameter_sensor_coordinate;
        parameter_sensor_coordinate.type = SMARTPORT_LATITUDE;
        parameter_sensor_coordinate.latitude = parameter.lat;
        parameter_sensor_coordinate.longitude = parameter.lon;
        parameter_sensor_coordinate.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_coordinates_task, "sensor_coordinates_task", STACK_SENSOR_SMARTPORT,
                    (void *)&parameter_sensor_coordinate, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_datetime_parameters_t parameter_sensor_datetime;
        parameter_sensor_datetime.type = SMARTPORT_DATE;
        parameter_sensor_datetime.date = parameter.date;
        parameter_sensor_datetime.time = parameter.time;
        parameter_sensor_datetime.rate = 1000;
        xTaskCreate(sensor_datetime_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor_datetime, 3,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = GPS_ALT_FIRST_ID;
        parameter_sensor.value = parameter.alt;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GPS_SPEED_FIRST_ID;
        parameter_sensor.value = parameter.spd;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GPS_COURS_FIRST_ID;
        parameter_sensor.value = parameter.cog;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID + 1;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 3;
        parameter_sensor.value = parameter.sat;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 4;
        parameter_sensor.value = parameter.dist;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 5;
        parameter_sensor.value = parameter.pdop;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = A3_FIRST_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = CURR_FIRST_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = NULL;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_current;
        xTaskCreate(sensor_double_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor_double, 3,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.ntc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 100,
                                           (float)config->airspeed_slope / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = AIR_SPEED_FIRST_ID;
        parameter_sensor.value = parameter.airspeed;
        parameter_sensor.rate = config->refresh_rate_airspeed;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = GASSUIT_FLOW_FIRST_ID;
        parameter_sensor.value = parameter.consumption_instant;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GASSUIT_RES_VOL_FIRST_ID;
        parameter_sensor.value = parameter.consumption_total;
        parameter_sensor.rate = config->refresh_rate_default;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->gpio_mask) {
        gpio_parameters_t parameter = {config->gpio_mask, config->gpio_interval, malloc(sizeof(float))};
        xTaskCreate(gpio_task, "gpio_task", STACK_GPIO, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_gpio_parameters_t parameter_sensor;
        parameter_sensor.data_id = DIY_FIRST_ID;
        parameter_sensor.value = parameter.value;
        parameter_sensor.rate = config->gpio_interval;
        parameter_sensor.gpio_mask = config->gpio_mask;
        xTaskCreate(sensor_gpio_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3,
                    &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

static uint8_t sensor_id_to_crc(uint8_t sensor_id) {
    if (sensor_id < 1 || sensor_id > 28) {
        return 0;
    }
    return sensor_id_matrix[sensor_id - 1];
}

static uint8_t sensor_crc_to_id(uint8_t sensor_id_crc) {
    uint8_t cont = 0;
    while (sensor_id_crc != sensor_id_matrix[cont] && cont < 28) {
        cont++;
    }
    if (cont == 28) return 0;
    return cont + 1;
}

static uint8_t get_crc(uint8_t *data) {
    uint16_t crc = 0;
    for (uint8_t i = 2; i < 9; i++) {
        crc += data[i];
        crc += crc >> 8;
        crc &= 0x00FF;
    }
    return 0xFF - (uint8_t)crc;
}
