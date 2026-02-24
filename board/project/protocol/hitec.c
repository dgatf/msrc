#include "hitec.h"

#include <math.h>
#include <pico/i2c_slave.h>
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
#include "esc_omp_m4.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "gps.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "ms5611.h"
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

static sensor_hitec_t *sensor;
static uint8_t packet[FRAME_LENGTH] = {0};
static volatile uint8_t cont = 0;
static volatile alarm_id_t alarm_id_recovery = 0;  // For bus recovery timeout
static volatile alarm_id_t alarm_id_reinit = 0;    // For mandatory deinit/init after each frame

static void i2c_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
static void set_config();
static int64_t alarm_recovery(alarm_id_t id, void *user_data);  // I2C bus recovery alarm
static int64_t alarm_reinit(alarm_id_t id, void *user_data);    // Periodic deinit/init after each frame
static int next_frame(void);
static void format_packet(uint8_t frame, uint8_t *buffer);
static void i2c_bus_recovery(void);

void hitec_task(void *parameters) {
    sensor = malloc(sizeof(sensor_hitec_t));
    *sensor =
        (sensor_hitec_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};

    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    set_config();

    i2c_bus_recovery();
    // Arm initial I2C bus watchdog: if no requests arrive soon after startup,
    // perform another bus recovery to handle a receiver that powers up later.
    alarm_id_recovery = add_alarm_in_us(1000 * 1000, alarm_recovery, NULL, true);

    debug("\nHitec init");

    vTaskSuspend(NULL);
}

void hitec_i2c_handler(void) {
    for (uint i = 0; i < FRAME_LENGTH + 1; i++) {
        i2c_handler(i2c1, I2C_SLAVE_REQUEST);
    }
}

static void i2c_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_REQUEST:
            if (cont == 0) {
                int frame = next_frame();
                if (frame < 0) {
                    return;
                }
                format_packet((uint8_t)frame, packet);
            }

            if (cont < FRAME_LENGTH) {
                // Send next byte
                i2c_write_byte_raw(i2c1, packet[cont++]);

                // Refresh I2C bus watchdog timeout (1 s).
                // As long as requests keep coming, the watchdog will never fire.
                if (alarm_id_recovery != 0) {
                    cancel_alarm(alarm_id_recovery);
                }
                alarm_id_recovery = add_alarm_in_us(1000 * 1000, alarm_recovery, NULL, true);

                // If we have just sent the last valid byte of the frame,
                // schedule a deinit/init after a short delay.
                if (cont == FRAME_LENGTH) {
                    // This handles the extra dummy read from Hitec master.
                    if (alarm_id_reinit != 0) {
                        cancel_alarm(alarm_id_reinit);
                    }
                    // 5 ms after frame end should be enough for master to finish its extra read
                    alarm_id_reinit = add_alarm_in_us(5 * 1000, alarm_reinit, NULL, true);

                    debug("\nHitec (%u) > ", uxTaskGetStackHighWaterMark(context.receiver_task_handle));
                    debug_buffer(packet, FRAME_LENGTH, "0x%X ");
                }
            } else {
                // Extra read from Hitec master: we just NACK and reset frame counter.
                cont = 0;
            }
            break;
    }
    vTaskResume(context.led_task_handle);
}

static int64_t alarm_reinit(alarm_id_t id, void *user_data) {
    // Mark current reinit alarm as consumed
    alarm_id_reinit = 0;

    // Short deinit/init sequence after each telemetry frame.
    // This is required because Hitec master always performs one extra read,
    // leaving the RP2040 I2C slave state machine in a bad state.
    i2c_slave_deinit(i2c1);
    sleep_us(5);  // Small delay to ensure peripheral is fully stopped
    i2c_slave_init(i2c1, I2C_ADDRESS, i2c_handler);

    // Reset frame counter just in case
    cont = 0;

    return 0;
}

static int64_t alarm_recovery(alarm_id_t id, void *user_data) {
    // I2C bus watchdog callback.
    // This is only reached if no I2C requests have been seen for the whole timeout
    // interval (1 s), because each request cancels and re-arms the watchdog.
    // While the bus is inactive/stuck, this function will keep running once per
    // second until a new request arrives and the handler cancels the alarm.
    i2c_bus_recovery();
    return 1000 * 1000;  // Keep watchdog running: retry bus recovery every 1 s
}


static void i2c_bus_recovery(void) {
    // Reset frame
    cont = 0;

    // Deinitialize I2C peripheral to release control of the bus
    i2c_slave_deinit(i2c1);

    // SCL as output, SDA as input with pull-up
    gpio_init(I2C1_SCL_GPIO);
    gpio_set_dir(I2C1_SCL_GPIO, GPIO_OUT);
    gpio_put(I2C1_SCL_GPIO, 1);

    gpio_init(I2C1_SDA_GPIO);
    gpio_set_dir(I2C1_SDA_GPIO, GPIO_IN);
    gpio_pull_up(I2C1_SDA_GPIO);

    sleep_us(5);

    // If SDA is low, someone is holding it: give up to 9 pulses on SCL
    for (int i = 0; i < 9 && !gpio_get(I2C1_SDA_GPIO); i++) {
        gpio_put(I2C1_SCL_GPIO, 0);
        sleep_us(5);
        gpio_put(I2C1_SCL_GPIO, 1);
        sleep_us(5);
    }

    // Generate STOP: SDA goes high while SCL is high
    // First ensure SDA is output and low
    gpio_set_dir(I2C1_SDA_GPIO, GPIO_OUT);
    gpio_put(I2C1_SDA_GPIO, 0);
    gpio_put(I2C1_SCL_GPIO, 1);
    sleep_us(5);
    gpio_put(I2C1_SDA_GPIO, 1);
    sleep_us(5);

    // Reinitialize I2C peripheral after bus recovery
    gpio_set_function(I2C1_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_GPIO);
    gpio_pull_up(I2C1_SCL_GPIO);

    // Reinitialize the slave
    i2c_slave_init(i2c1, I2C_ADDRESS, i2c_handler);
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
                double degrees = *sensor->frame_0x12[FRAME_0X12_GPS_LAT];
                int8_t deg = degrees;
                int8_t min = (degrees - deg) * 60;
                double sec = ((degrees - deg) * 60 - min) * 60;
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
                float degrees = *sensor->frame_0x13[FRAME_0X13_GPS_LON];
                int8_t deg = degrees;
                int8_t min = (degrees - deg) * 60;
                float sec = ((degrees - deg) * 60 - min) * 60;
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
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->is_enabled_frame[FRAME_0X15] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (config->enable_pwm_out) {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            context.pwm_out_task_handle = task_handle;
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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;

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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temperature_fet;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;

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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temp_esc;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;

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

        sensor->frame_0x15[FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.temp_esc;
        sensor->is_enabled_frame[FRAME_0X15] = true;
        sensor->is_enabled_frame[FRAME_0X18] = true;
        sensor->is_enabled_frame[FRAME_0X14] = true;

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

        sensor->frame_0x18[FRAME_0X18_AMP] = parameter.current;
        sensor->is_enabled_frame[FRAME_0X18] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);

        sensor->frame_0x14[FRAME_0X14_TEMP1] = parameter.ntc;
        sensor->is_enabled_frame[FRAME_0X14] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor->frame_0x1B[FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor->frame_0x1B[FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);

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
                                           (float)config->airspeed_offset / 1000,
                                           (float)config->airspeed_vcc / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);

        sensor->frame_0x1A[FRAME_0X1A_ASPD] = parameter.airspeed;
        sensor->is_enabled_frame[FRAME_0X1A] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}