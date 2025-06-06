#include "hitec.h"

#include <math.h>
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
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "gps.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "smart_esc.h"
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

// Frame 7 bytes: <frame id> <5 bytes payload> <frame id>

// hitec_frame_0x11_t internal telemetry rx batt
typedef struct hitec_frame_0x11_t {
    uint8_t b1;        // 0xAF
    uint8_t b2;        // 0x0
    uint8_t b3;        // 0x0
    uint16_t rx_batt;  // v * 28
} __attribute__((packed)) hitec_frame_0x11_t;

typedef struct hitec_frame_0x12_t {  // lat, time
    uint16_t sec;                    // s * 100
    uint16_t deg_min;                // ddmm
    uint8_t seconds;                 // time seconds
} __attribute__((packed)) hitec_frame_0x12_t;

typedef struct hitec_frame_0x13_t {  // lon, temp2
    uint16_t sec;                    // s * 100
    uint16_t deg_min;                // ddmm
    uint8_t temp2;                   // t + 40 C
} __attribute__((packed)) hitec_frame_0x13_t;

typedef struct hitec_frame_0x14_t {  // spd, alt, temp1
    uint16_t spd;                    // kmh
    int16_t alt;                     //
    uint8_t temp1;                   // t + 40 C
} __attribute__((packed)) hitec_frame_0x14_t;

typedef struct hitec_frame_0x15_t {  // rpm
    uint8_t unused;
    uint16_t rpm1;
    uint16_t rpm2;
} __attribute__((packed)) hitec_frame_0x15_t;

typedef struct hitec_frame_0x16_t {  // date, time
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hour;
    uint8_t min;
} __attribute__((packed)) hitec_frame_0x16_t;

typedef struct hitec_frame_0x17_t {  // cog, sats, tem3, temp4
    uint16_t cog;
    uint8_t sats;
    uint8_t temp3;
    uint8_t temp4;
} __attribute__((packed)) hitec_frame_0x17_t;

typedef struct hitec_frame_0x18_t {  // volt, amp
    uint16_t volt;
    uint16_t amp;
    uint8_t unused;
} __attribute__((packed)) hitec_frame_0x18_t;

typedef struct hitec_frame_0x19_t {  // amp
    uint8_t amp1;
    uint8_t amp2;
    uint8_t amp3;
    uint8_t amp4;
} __attribute__((packed)) hitec_frame_0x19_t;

typedef struct hitec_frame_0x1A_t {  // airspeed
    uint16_t airspeed;
    uint16_t unused1;
    uint8_t unused2;
} __attribute__((packed)) hitec_frame_0x1A_t;

typedef struct hitec_frame_0x1B_t {  // altitude
    uint16_t alt_u;
    uint16_t alt_f;  // unused
    uint8_t unused;
} __attribute__((packed)) hitec_frame_0x1B_t;


typedef struct hitec_sensor_frame_0x12_t {  // lat, time
    double *lat;
    float *time;                  
} hitec_sensor_frame_0x12_t;

typedef struct hitec_sensor_frame_0x13_t {  // lon, temp2
    double *lon;            
    float *temp2;                 
} hitec_sensor_frame_0x13_t;

typedef struct hitec_sensor_frame_0x14_t {  // spd, alt, temp1
    float *spd;                   
    float *alt;                     
    float *temp1;                  
} hitec_sensor_frame_0x14_t;

typedef struct hitec_sensor_frame_0x15_t {  // rpm
    float *rpm1;
    float *rpm2;
} hitec_sensor_frame_0x15_t;

typedef struct hitec_sensor_frame_0x16_t {  // date, time
    float *date;
    float *time;
} hitec_sensor_frame_0x16_t;

typedef struct hitec_sensor_frame_0x17_t {  // cog, sats, tem3, temp4
    float *cog;
    float *sats;
    float *temp3;
    float *temp4;
} hitec_sensor_frame_0x17_t;

typedef struct hitec_sensor_frame_0x18_t {  // volt, amp
    float *volt;
    float *amp;
} hitec_sensor_frame_0x18_t;

typedef struct hitec_sensor_frame_0x19_t {  // amp
    float *amp1;
    float *amp2;
    float *amp3;
    float *amp4;
} hitec_sensor_frame_0x19_t;

typedef struct hitec_sensor_frame_0x1A_t {  // airspeed
    float *airspeed;
} hitec_sensor_frame_0x1A_t;

typedef struct hitec_sensor_frame_0x1B_t {  // altitude
    float *alt_u;
} hitec_sensor_frame_0x1B_t;

typedef struct hitec_sensors_t {
    bool is_enabled_frame[11];
    hitec_sensor_frame_0x12_t frame_0x12;
    hitec_sensor_frame_0x13_t frame_0x13;
    hitec_sensor_frame_0x14_t frame_0x14;
    hitec_sensor_frame_0x15_t frame_0x15;
    hitec_sensor_frame_0x16_t frame_0x16;
    hitec_sensor_frame_0x17_t frame_0x17;
    hitec_sensor_frame_0x18_t frame_0x18;
    hitec_sensor_frame_0x19_t frame_0x19;
    hitec_sensor_frame_0x1A_t frame_0x1A;
    hitec_sensor_frame_0x1B_t frame_0x1B;
} hitec_sensors_t;

static volatile hitec_sensors_t *sensors;

static void i2c_request_handler(uint8_t address);
static void i2c_stop_handler(uint8_t length);
static void set_config(void);
static int next_frame(void);
static void format_packet(uint8_t frame, uint8_t *buffer);

void hitec_i2c_handler(void) { i2c_request_handler(I2C_ADDRESS); }

void hitec_task(void *parameters) {
    sensors = calloc(1, sizeof(hitec_sensors_t));

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
    //i2c_multi_set_write_buffer(buffer);

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
    while (!sensors->is_enabled_frame[frame] && cont < 12) {
        frame++;
        frame %= 11;
        cont++;
    }
    if (cont == 12) return -1;
    return frame;
}

static void format_packet(uint8_t frame, uint8_t *buffer) {
    // big endian packets
    buffer[0] = frame + 0x11;
    buffer[6] = frame + 0x11;
    switch (frame) {
        case FRAME_0X12: {
            hitec_frame_0x12_t msg = {0};
            double degrees = *sensors->frame_0x12.lat;
            int8_t deg = degrees;
            int8_t min = (degrees - deg) * 60;
            double sec = ((degrees - deg) * 60 - min) * 60;
            int16_t sec_x_100 = sec * 100;
            int16_t deg_min = deg * 100 + min;
            msg.sec = swap_16(sec_x_100);
            msg.deg_min = swap_16(deg_min);
            msg.seconds = *sensors->frame_0x12.time;
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X13: {
            hitec_frame_0x13_t msg = {0};
            double degrees = *sensors->frame_0x13.lon;
            int8_t deg = degrees;
            int8_t min = (degrees - deg) * 60;
            double sec = ((degrees - deg) * 60 - min) * 60;
            int16_t sec_x_100 = sec * 100;
            int16_t deg_min = deg * 100 + min;
            msg.sec = swap_16(sec_x_100);
            msg.deg_min = swap_16(deg_min);
            msg.temp2 = round(*sensors->frame_0x13.temp2 + 40);
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X14: {
            hitec_frame_0x14_t msg = {0};
            msg.spd = swap_16((uint16_t)round(*sensors->frame_0x14.spd * 1.852));
            msg.alt = round(*sensors->frame_0x14.alt);
            msg.temp1 = round(*sensors->frame_0x14.temp1 + 40);
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X15: {
            hitec_frame_0x15_t msg = {0};
            msg.rpm1 = swap_16((uint16_t)round(*sensors->frame_0x15.rpm1));
            msg.rpm2 = swap_16((uint16_t)round(*sensors->frame_0x15.rpm2));
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X16: {
            hitec_frame_0x16_t msg = {0};
            int32_t value = *sensors->frame_0x16.date;
            msg.year = value / 10000;                                 // year
            msg.month = (value - buffer[3] * 10000UL) / 100;          // month
            msg.day = value - buffer[3] * 10000UL - buffer[2] * 100;  // day
            value = *sensors->frame_0x16.time;
            msg.hour = value / 10000;                       // hour
            msg.min = (value - buffer[4] * 10000UL) / 100;  // minute
            memcpy(buffer + 1, &msg, sizeof(msg));
            debug("\nDT: %f", *sensors->frame_0x16.date);
            break;
        }
        case FRAME_0X17: {
            hitec_frame_0x17_t msg = {0};
            msg.cog = swap_16((uint16_t)round(*sensors->frame_0x17.cog));
            msg.sats = *sensors->frame_0x17.sats;
            msg.temp3 = round(*sensors->frame_0x17.temp3 + 40);
            msg.temp4 = round(*sensors->frame_0x17.temp4 + 40);
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X18: {
            hitec_frame_0x18_t msg = {0};
            msg.volt = swap_16((uint16_t)round((*sensors->frame_0x18.volt - 0.2) * 10));

            /* value for stock transmitter (tbc) */
            msg.amp = swap_16((uint16_t)((*sensors->frame_0x18.amp + 114.875) * 1.441));

            /* value for opentx transmitter  */
            // msg.amp = round(*sensors->frame_0x18.amp);
            
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X19: {
            hitec_frame_0x19_t msg = {0};
            msg.amp1 = round(*sensors->frame_0x19.amp1 * 10);
            msg.amp2 = round(*sensors->frame_0x19.amp2 * 10);
            msg.amp3 = round(*sensors->frame_0x19.amp3 * 10);
            msg.amp4 = round(*sensors->frame_0x19.amp4 * 10);
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X1A: {
            hitec_frame_0x1A_t msg = {0};
            msg.airspeed = swap_16((uint16_t)round(*sensors->frame_0x1A.airspeed));
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
        case FRAME_0X1B: {
            hitec_frame_0x1B_t msg = {0};
            msg.alt_u = round(*sensors->frame_0x1B.alt_u);
            memcpy(buffer + 1, &msg, sizeof(msg));
            break;
        }
    }
}

static void set_config(void) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    hitec_sensors_t *new_sensor;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->is_enabled_frame[FRAME_0X15] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->is_enabled_frame[FRAME_0X15] = true;

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
        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature_fet;
        sensors->frame_0x13.temp2 = parameter.temperature_bec;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;
        sensors->is_enabled_frame[FRAME_0X13] = true;
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
        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature_fet;
        sensors->frame_0x13.temp2 = parameter.temperature_bec;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;
        sensors->is_enabled_frame[FRAME_0X13] = true;
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
        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;
        sensors->is_enabled_frame[FRAME_0X13] = true;

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

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature_fet;
        sensors->frame_0x13.temp2 = parameter.temperature_bec;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;
        sensors->is_enabled_frame[FRAME_0X13] = true;

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

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;

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

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;

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
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temperature_fet;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;

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

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temp_esc;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;

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
        xTaskCreate(esc_omp_m4_task, "esc_omp_m4_task", STACK_ESC_OMP_M4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->frame_0x15.rpm1 = parameter.rpm;
        sensors->frame_0x18.volt = parameter.voltage;
        sensors->frame_0x18.amp = parameter.current;
        sensors->frame_0x14.temp1 = parameter.temp_esc;
        sensors->is_enabled_frame[FRAME_0X15] = true;
        sensors->is_enabled_frame[FRAME_0X18] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;

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
        parameter.dist = malloc(sizeof(float));
        xTaskCreate(gps_task, "gps_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;
        
        sensors->frame_0x17.sats = parameter.sat;
        sensors->frame_0x12.lat = parameter.lat;
        sensors->frame_0x13.lon = parameter.lon;
        sensors->frame_0x14.alt = parameter.alt;
        sensors->frame_0x14.spd = parameter.spd;
        sensors->frame_0x17.cog = parameter.cog;
        sensors->frame_0x16.date = parameter.date;
        sensors->frame_0x16.date = parameter.time;
        sensors->is_enabled_frame[FRAME_0X17] = true;
        sensors->is_enabled_frame[FRAME_0X12] = true;
        sensors->is_enabled_frame[FRAME_0X13] = true;
        sensors->is_enabled_frame[FRAME_0X14] = true;
        sensors->is_enabled_frame[FRAME_0X16] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->frame_0x18.volt = parameter.voltage;
        sensors->is_enabled_frame[FRAME_0X18] = true;

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

        sensors->frame_0x18.amp = parameter.current;
        sensors->is_enabled_frame[FRAME_0X18] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->frame_0x14.temp1 = parameter.ntc;
        sensors->is_enabled_frame[FRAME_0X14] = true;

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

        sensors->frame_0x1B.alt_u = parameter.altitude;
        sensors->is_enabled_frame[FRAME_0X1B] = true;

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

        sensors->frame_0x1B.alt_u = parameter.altitude;
        sensors->is_enabled_frame[FRAME_0X1B] = true;

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

        sensors->frame_0x1B.alt_u = parameter.altitude;
        sensors->is_enabled_frame[FRAME_0X1B] = true;

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

        sensors->frame_0x1A.airspeed = parameter.airspeed;
        sensors->is_enabled_frame[FRAME_0X1A] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
