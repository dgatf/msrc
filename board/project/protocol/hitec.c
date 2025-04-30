#include "hitec.h"

#include <math.h>
#include <stdio.h>

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
#include "esc_pwm.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define I2C_INTR_MASK_RD_REQ 0x00000020

#define I2C_ADDRESS 0x08
#define TIMEOUT 1000
#define FRAME_LENGTH 7

#define FRAME_0X11 0
#define FRAME_0X12 1
#define FRAME_0X13 2
#define FRAME_0X14 3
#define FRAME_0X15 4
#define FRAME_0X16 5
#define FRAME_0X17 6
#define FRAME_0X18 7
#define FRAME_0X19 8
#define FRAME_0X1A 9
#define FRAME_0X1B 10
#define FRAME_0X11_RX_BATT 0
#define FRAME_0X12_GPS_LAT 0
#define FRAME_0X12_TIME 1
#define FRAME_0X13_GPS_LON 0
#define FRAME_0X13_TEMP2 1
#define FRAME_0X14_GPS_SPD 0
#define FRAME_0X14_GPS_ALT 1
#define FRAME_0X14_TEMP1 2
#define FRAME_0X15_FUEL 0
#define FRAME_0X15_RPM1 1
#define FRAME_0X15_RPM2 2
#define FRAME_0X16_DATE 0
#define FRAME_0X16_TIME 1
#define FRAME_0X17_COG 0
#define FRAME_0X17_SATS 1
#define FRAME_0X17_TEMP3 2
#define FRAME_0X17_TEMP4 3
#define FRAME_0X18_VOLT 0
#define FRAME_0X18_AMP 1
#define FRAME_0X19_AMP1 0
#define FRAME_0X19_AMP2 1
#define FRAME_0X19_AMP3 2
#define FRAME_0X19_AMP4 3
#define FRAME_0X1A_ASPD 0
#define FRAME_0X1B_ALTU 0
#define FRAME_0X1B_ALTF 1

static sensor_hitec_t *sensor;

static void i2c_request_handler(uint8_t address);
static void i2c_stop_handler(uint8_t length);
static void set_config(void);
static int next_frame(void);
static void format_packet(uint8_t frame, uint8_t *buffer);

void hitec_i2c_handler(void) { i2c_request_handler(I2C_ADDRESS); }

void hitec_task(void *parameters) {
    sensor = malloc(sizeof(sensor_hitec_t));
    *sensor =
        (sensor_hitec_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};

    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    set_config();

    PIO pio = pio1;
    uint pin = I2C1_SDA_GPIO;
    i2c_multi_init(pio, pin);
    i2c_multi_set_request_handler(i2c_request_handler);
    i2c_multi_set_stop_handler(i2c_stop_handler);
    i2c_multi_enable_address(I2C_ADDRESS);
    i2c_multi_fixed_length(FRAME_LENGTH);
    gpio_set_drive_strength(I2C1_SDA_GPIO, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C1_SDA_GPIO + 1, GPIO_DRIVE_STRENGTH_12MA);

    debug("\nHitec init");
    vTaskSuspend(NULL);
}

static void i2c_stop_handler(uint8_t length) { debug(" - STOP (%u)", length); }

static void i2c_request_handler(uint8_t address) {
    uint8_t buffer[7] = {0};
    uint8_t frame = next_frame();
    if (frame < 0) return;
    format_packet(frame, buffer);
    i2c_multi_set_write_buffer(buffer);

    // blink led
    vTaskResume(context.led_task_handle);

    debug("\nHitec (%u) > ", uxTaskGetStackHighWaterMark(context.receiver_task_handle));
    debug_buffer(buffer, sizeof(buffer), "%X ");
}

static int next_frame(void) {
    static uint8_t frame = 0;
    uint cont = 0;
    frame++;
    frame %= 11;
    while (!sensor->is_enabled_frame[frame] && cont < 12) {
        frame++;
        frame %= 11;
        cont++;
    }
    if (cont == 12) return -1;
    return frame;
}

static void format_packet(uint8_t frame, uint8_t *buffer) {
    int32_t valueS32;
    uint16_t valueU16;
    uint16_t valueS16;
    uint8_t valueU8;
    buffer[0] = frame + 0x11;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = frame + 0x11;
    switch (frame) {
        case FRAME_0X11:
            buffer[1] = 0xAF;
            buffer[3] = 0x2D;
            if (sensor->frame_0x11[FRAME_0X11_RX_BATT]) {
                valueU16 = *sensor->frame_0x11[FRAME_0X11_RX_BATT] * 28;
                buffer[4] = valueU16 >> 8;
                buffer[5] = valueU16;
            }
            break;
        case FRAME_0X12:
            if (sensor->frame_0x12[FRAME_0X12_GPS_LAT]) {
                float degF = *sensor->frame_0x12[FRAME_0X12_GPS_LAT] / 60;
                int8_t deg = degF;
                int8_t min = (degF - deg) * 60;
                float sec = ((degF - deg) * 60 - min) * 60;
                int16_t sec_x_100 = sec * 100;
                int16_t deg_min = deg * 100 + min;
                buffer[1] = sec_x_100 >> 8;
                buffer[2] = sec_x_100;
                buffer[3] = deg_min >> 8;
                buffer[4] = deg_min;
            }
            if (sensor->frame_0x12[FRAME_0X12_TIME]) {
                valueU8 = *sensor->frame_0x12[FRAME_0X12_TIME];
            }
            break;
        case FRAME_0X13:
            if (sensor->frame_0x13[FRAME_0X13_GPS_LON]) {
                float degF = *sensor->frame_0x13[FRAME_0X13_GPS_LON] / 60;
                int8_t deg = degF;
                int8_t min = (degF - deg) * 60;
                float sec = ((degF - deg) * 60 - min) * 60;
                int16_t sec_x_100 = sec * 100;
                int16_t deg_min = deg * 100 + min;
                buffer[1] = sec_x_100 >> 8;
                buffer[2] = sec_x_100;
                buffer[3] = deg_min >> 8;
                buffer[4] = deg_min;
            }
            if (sensor->frame_0x13[FRAME_0X13_TEMP2]) {
                valueU8 = round(*sensor->frame_0x13[FRAME_0X13_TEMP2] + 40);
                buffer[5] = valueU8;
            }
            break;
        case FRAME_0X14:
            if (sensor->frame_0x14[FRAME_0X14_GPS_SPD]) {
                valueU16 = round(*sensor->frame_0x14[FRAME_0X14_GPS_SPD] * 1.852);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (sensor->frame_0x14[FRAME_0X14_GPS_ALT]) {
                valueS16 = round(*sensor->frame_0x14[FRAME_0X14_GPS_ALT]);
                buffer[3] = valueS16 >> 8;
                buffer[4] = valueS16;
            }
            if (sensor->frame_0x14[FRAME_0X14_TEMP1]) {
                valueU8 = round(*sensor->frame_0x14[FRAME_0X14_TEMP1] + 40);
                buffer[5] = valueU8;
            }
            break;
        case FRAME_0X15:
            if (sensor->frame_0x15[FRAME_0X15_RPM1]) {
                valueU16 = round(*sensor->frame_0x15[FRAME_0X15_RPM1]);
                buffer[2] = valueU16;
                buffer[3] = valueU16 >> 8;
            }
            if (sensor->frame_0x15[FRAME_0X15_RPM2]) {
                valueU16 = round(*sensor->frame_0x15[FRAME_0X15_RPM2]);
                buffer[4] = valueU16;
                buffer[5] = valueU16 >> 8;
            }
            break;
        case FRAME_0X16:
            if (sensor->frame_0x16[FRAME_0X16_DATE]) {
                valueS32 = *sensor->frame_0x16[FRAME_0X16_DATE];
                buffer[3] = valueS32 / 10000;                                  // year
                buffer[2] = (valueS32 - buffer[3] * 10000UL) / 100;            // month
                buffer[1] = valueS32 - buffer[3] * 10000UL - buffer[2] * 100;  // day
            }
            if (sensor->frame_0x16[FRAME_0X16_TIME]) {
                valueS32 = *sensor->frame_0x16[FRAME_0X16_TIME];
                buffer[4] = valueS32 / 10000;                        // hour
                buffer[5] = (valueS32 - buffer[4] * 10000UL) / 100;  // minute
            }
            break;
        case FRAME_0X17:
            if (sensor->frame_0x17[FRAME_0X17_COG]) {
                valueU16 = round(*sensor->frame_0x17[FRAME_0X17_COG]);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (sensor->frame_0x17[FRAME_0X17_SATS]) {
                valueU8 = *sensor->frame_0x17[FRAME_0X17_SATS];
                buffer[3] = valueU8;
            }
            if (sensor->frame_0x17[FRAME_0X17_TEMP3]) {
                valueU8 = round(*sensor->frame_0x17[FRAME_0X17_TEMP3] + 40);
                buffer[4] = valueU8;
            }
            if (sensor->frame_0x17[FRAME_0X17_TEMP4]) {
                valueU8 = round(*sensor->frame_0x17[FRAME_0X17_TEMP4] + 40);
                buffer[5] = valueU8;
            }
            break;
        case FRAME_0X18:
            if (sensor->frame_0x18[FRAME_0X18_VOLT]) {
                valueU16 = round((*sensor->frame_0x18[FRAME_0X18_VOLT] - 0.2) * 10);
                buffer[1] = valueU16;
                buffer[2] = valueU16 >> 8;
            }
            if (sensor->frame_0x18[FRAME_0X18_AMP]) {
                /* value for stock transmitter (tbc) */
                // valueU16 = (*sensor->frame_0x18[FRAME_0X18_AMP] + 114.875) * 1.441;

                /* value for opentx transmitter  */
                valueU16 = round(*sensor->frame_0x18[FRAME_0X18_AMP]);

                buffer[3] = valueU16;
                buffer[4] = valueU16 >> 8;
            }
            break;
        case FRAME_0X19:
            if (sensor->frame_0x19[FRAME_0X19_AMP1]) {
                valueU8 = round(*sensor->frame_0x19[FRAME_0X19_AMP1] * 10);
                buffer[5] = valueU8;
            }
            if (sensor->frame_0x19[FRAME_0X19_AMP2]) {
                valueU8 = round(*sensor->frame_0x19[FRAME_0X19_AMP2] * 10);
                buffer[5] = valueU8;
            }
            if (sensor->frame_0x19[FRAME_0X19_AMP3]) {
                valueU8 = round(*sensor->frame_0x19[FRAME_0X19_AMP3] * 10);
                buffer[5] = valueU8;
            }
            if (sensor->frame_0x19[FRAME_0X19_AMP4]) {
                valueU8 = round(*sensor->frame_0x19[FRAME_0X19_AMP4] * 10);
                buffer[5] = valueU8;
            }
            break;
        case FRAME_0X1A:
            if (sensor->frame_0x1A[FRAME_0X1A_ASPD]) {
                valueU16 = round(*sensor->frame_0x1A[FRAME_0X1A_ASPD]);
                buffer[3] = valueU16 >> 8;
                buffer[4] = valueU16;
            }
            break;
        case FRAME_0X1B:
            if (sensor->frame_0x1B[FRAME_0X1B_ALTU]) {
                valueU16 = round(*sensor->frame_0x1B[FRAME_0X1B_ALTU]);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (sensor->frame_0x1B[FRAME_0X1B_ALTF]) {
                valueU16 = round(*sensor->frame_0x1B[FRAME_0X1B_ALTF]);
                buffer[3] = valueU16 >> 8;
                buffer[4] = valueU16;
            }
            break;
    }
}

static void set_config(void) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_hitec_t *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->is_enabled_frame[FRAME_0X15] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->is_enabled_frame[FRAME_0X15] = true;

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
        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature_fet;
        sensor->frame_0x13[FRAME_0X13_TEMP2] = parameter.temperature_bec;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;
        sensor->is_enabled_frame[FRAME_0X13] = true;
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
        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature_fet;
        sensor->frame_0x13[FRAME_0X13_TEMP2] = parameter.temperature_bec;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;
        sensor->is_enabled_frame[FRAME_0X13] = true;
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
        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;
        sensor->is_enabled_frame[FRAME_0X13] = true;

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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature_fet;
        sensor->frame_0x13[FRAME_0X13_TEMP2] = parameter.temperature_bec;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;
        sensor->is_enabled_frame[FRAME_0X13] = true;

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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;

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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps) {
        nmea_parameters_t parameter = {config->gps_baudrate,  malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;

        sensor->frame_0x17[FRAME_0X17_SATS] = parameter.sat;
        sensor->frame_0x12[FRAME_0X12_GPS_LAT] = parameter.lat;
        sensor->frame_0x13[FRAME_0X13_GPS_LON] = parameter.lon;
        sensor->frame_0x14[FRAME_0X14_GPS_ALT] = parameter.alt;
        sensor->frame_0x14[FRAME_0X14_GPS_SPD] = parameter.spd;
        sensor->frame_0x17[FRAME_0X17_COG] = parameter.cog;
        sensor->frame_0x16[FRAME_0X16_DATE] = parameter.date;
        sensor->frame_0x16[FRAME_0X16_TIME] = parameter.time;
        sensor->is_enabled_frame[FRAME_0X17] = true;
        sensor->is_enabled_frame[FRAME_0X12] = true;
        sensor->is_enabled_frame[FRAME_0X13] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;
        sensor->is_enabled_frame[FRAME_0X16] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->is_enabled_frame[FRAME_0X18] = true;

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

        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->is_enabled_frame[FRAME_0X18] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.ntc;
        sensor->is_enabled_frame[FRAME_0X14] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor->frame_0x1B[FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor->frame_0x1B[FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor->frame_0x1B[FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[FRAME_0X1B] = true;

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

        sensor->frame_0x1A[FRAME_0X1A_ASPD] = parameter.airspeed;
        sensor->is_enabled_frame[FRAME_0X1A] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
