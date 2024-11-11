#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "airspeed.h"
#include "bmp180.h"
#include "bmp280.h"
#include "config.h"
#include "crsf.h"
#include "current.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "esc_hw5.h"
#include "esc_kontronik.h"
#include "esc_pwm.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "ibus.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pwm_out.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)
#define swap_24(value) (((value & 0xFF) << 16) | (value & 0xFF00) | (value & 0xFF0000) >> 16)
#define swap_32(value) \
    (((value & 0xFF) << 24) | ((value & 0xFF00) << 8) | ((value & 0xFF0000) >> 8) | ((value & 0xFF000000) >> 24))

#define SANWA_TIMEOUT_US 500

#define TYPE_TEMP_MOTOR 0
#define TYPE_TEMP_ESC 1
#define TYPE_UNKNOWN 2
#define TYPE_RPM1 3
#define TYPE_RPM2 4
#define TYPE_VOLT 5

#define MAX_SENSORS 6

#define TELEMETRY_PACKET_LENGHT 5
#define CHANNEL_PACKET_LENGHT 10

#define GPIO_PWM_CH1 10
#define GPIO_PWM_CH2 11

#define PWM_FREQ 20  // ms

typedef struct sanwa_sensor_formatted_t {
    uint8_t header;
    uint8_t type;
    uint8_t msb;
    uint8_t lsb;
    uint8_t crc;
} __attribute__((packed)) sanwa_sensor_formatted_t;

static void process(float **sensors);
static void format_sensor(float *sensor, uint8_t type, sanwa_sensor_formatted_t *sensor_formatted);
static void send_packet(float **sensors);
static uint8_t get_crc(const uint8_t *buffer, uint len);
static void set_config(float **sensors);

void sanwa_task(void *parameters) {
    /*gpio_set_function(GPIO_PWM_CH1, GPIO_FUNC_PWM);
    gpio_set_function(GPIO_PWM_CH2, GPIO_FUNC_PWM);
    uint slice_num_ch1 = pwm_gpio_to_slice_num(GPIO_PWM_CH1);
    uint slice_num_ch2 = pwm_gpio_to_slice_num(GPIO_PWM_CH2);
    pwm_config config_ch1 = pwm_get_default_config();
    pwm_config config_ch2 = pwm_get_default_config();
    uint clk_div =
        PWM_FREQ * clock_get_hz(clk_sys) / 1000 / 65536.0;  // clk_div = pulse_freq * clk_sys_hz / (1000 * 65536)
    pwm_config_set_wrap(&config_ch1, 0xFFFF);
    pwm_set_gpio_level(GPIO_PWM_CH1, 65536 / 20);  // set ch1 to lowest position
    pwm_set_gpio_level(GPIO_PWM_CH2, 65536 / 20);  // set ch2 to lowest position*/

    float *sensors[6] = {0};
    set_config(sensors);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(115200L, UART_RECEIVER_TX, UART_RECEIVER_RX, SANWA_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    debug("\nSanwa init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(sensors);
    }
}

static void process(float **sensors) {
    if (uart0_available() == CHANNEL_PACKET_LENGHT) {
        uint8_t buffer[CHANNEL_PACKET_LENGHT];
        uart0_read_bytes(buffer, CHANNEL_PACKET_LENGHT);
        debug("\nSanwa (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, CHANNEL_PACKET_LENGHT, "0x%X ");
        if (buffer[0] == 0x01 && get_crc(buffer, CHANNEL_PACKET_LENGHT - 1) == buffer[CHANNEL_PACKET_LENGHT - 1]) {
            // pwm_set_gpio_level(GPIO_PWM_CH1, (buffer[1] << 8) | buffer[2]);
            // pwm_set_gpio_level(GPIO_PWM_CH2, (buffer[3] << 8) | buffer[4]);
            send_packet(sensors);
        } else {
            debug("\nSanwa. Bad header");
        }
    }
}

static void send_packet(float **sensors) {
    if (!sensors[TYPE_TEMP_MOTOR] && !sensors[TYPE_TEMP_ESC] && !sensors[TYPE_RPM1] && !sensors[TYPE_RPM2] &&
        !sensors[TYPE_VOLT])
        return;
    static uint type = TYPE_TEMP_MOTOR;
    sanwa_sensor_formatted_t sensor_formatted = {0};
    while (!sensors[type % MAX_SENSORS]) type++;
    format_sensor(sensors[type % MAX_SENSORS], type % MAX_SENSORS, &sensor_formatted);
    uart0_write_bytes((uint8_t *)&sensor_formatted, TELEMETRY_PACKET_LENGHT);
    type++;
    debug("\nSanwa (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    debug_buffer((uint8_t *)&sensor_formatted, TELEMETRY_PACKET_LENGHT, "0x%X ");

    // blink led
    vTaskResume(context.led_task_handle);
}

static void format_sensor(float *sensor, uint8_t type, sanwa_sensor_formatted_t *sensor_formatted) {
    // Packet format: [sync] [type] [msb] [lsb] [crc8]
    static float rpm = 0;
    sensor_formatted->header = 0x01;
    sensor_formatted->type = type;
    switch (type) {
        case TYPE_TEMP_MOTOR: {
            static uint8_t prev = 0;
            sensor_formatted->msb = prev;
            sensor_formatted->lsb = *sensor;
            prev = sensor_formatted->lsb;
            sensor_formatted->crc = get_crc((uint8_t *)sensor_formatted, TELEMETRY_PACKET_LENGHT - 1);
            break;
        }
        case TYPE_TEMP_ESC: {
            static uint8_t prev = 0;
            sensor_formatted->msb = prev;
            sensor_formatted->lsb = *sensor;
            prev = sensor_formatted->lsb;
            sensor_formatted->crc = get_crc((uint8_t *)sensor_formatted, TELEMETRY_PACKET_LENGHT - 1);
            break;
        }
        case TYPE_RPM1: {
            rpm = *sensor;
            sensor_formatted->msb = (uint)(rpm / 2) >> 8;
            sensor_formatted->lsb = rpm / 2;
            sensor_formatted->crc = get_crc((uint8_t *)sensor_formatted, TELEMETRY_PACKET_LENGHT - 1);
            break;
        }
        case TYPE_RPM2: {
            sensor_formatted->msb = (uint)(rpm / 2) >> 8;
            sensor_formatted->lsb = rpm / 2;
            sensor_formatted->crc = get_crc((uint8_t *)sensor_formatted, TELEMETRY_PACKET_LENGHT - 1);
            break;
        }
        case TYPE_VOLT: {
            uint16_t volt = *sensor * 100;
            sensor_formatted->msb = volt >> 8;
            sensor_formatted->lsb = volt;
            sensor_formatted->crc = get_crc((uint8_t *)sensor_formatted, TELEMETRY_PACKET_LENGHT - 1);
            break;
        }
    }
}

static uint8_t get_crc(const uint8_t *buffer, uint len) {
    uint8_t crc = 0;
    for (uint i = 0; i < len; i++) crc += buffer[i];
    return crc;
}

static void set_config(float **sensors) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors[TYPE_RPM1] = parameter.rpm;
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
        /*if (config->enable_pwm_out) {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            context.pwm_out_task_handle = task_handle;
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }*/
        sensors[TYPE_TEMP_ESC] = parameter.temperature_fet;
        sensors[TYPE_TEMP_MOTOR] = parameter.temperature_bec;  // note this is bec temp, not motor temp
        sensors[TYPE_VOLT] = parameter.voltage;
        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
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

        sensors[TYPE_TEMP_ESC] = parameter.temperature_fet;
        sensors[TYPE_TEMP_MOTOR] = parameter.temperature_bec;  // note this is bec temp, not motor temp
        sensors[TYPE_VOLT] = parameter.voltage;
        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
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

        sensors[TYPE_TEMP_ESC] = parameter.temperature;
        sensors[TYPE_VOLT] = parameter.voltage;
        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
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

        sensors[TYPE_TEMP_ESC] = parameter.temperature_fet;
        sensors[TYPE_TEMP_MOTOR] = parameter.temperature_bec;  // note this is bec temp, not motor temp
        sensors[TYPE_VOLT] = parameter.voltage;
        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
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

        sensors[TYPE_TEMP_ESC] = parameter.temperature;
        sensors[TYPE_VOLT] = parameter.voltage;
        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
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

        sensors[TYPE_TEMP_ESC] = parameter.temperature;
        sensors[TYPE_VOLT] = parameter.voltage;
        sensors[TYPE_RPM1] = parameter.rpm;
        sensors[TYPE_RPM2] = parameter.rpm;
    }

    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors[TYPE_VOLT] = parameter.voltage;
    }

    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors[TYPE_TEMP_MOTOR] = parameter.ntc;
    }
}
