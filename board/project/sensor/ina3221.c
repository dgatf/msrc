#include "ina3221.h"

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

//  REGISTERS
#define INA3221_CONFIGURATION (0x00)
#define INA3221_SHUNT_VOLTAGE(x) (0x01 + (x * 2))
#define INA3221_BUS_VOLTAGE(x) (0x02 + (x * 2))
#define INA3221_CRITICAL_ALERT(x) (0x07 + (x * 2))
#define INA3221_WARNING_ALERT(x) (0x08 + (x * 2))
#define INA3221_SHUNT_VOLTAGE_SUM (0x0D)
#define INA3221_SHUNT_VOLTAGE_LIMIT (0x0E)
#define INA3221_MASK_ENABLE (0x0F)
#define INA3221_POWER_VALID_UPPER (0x10)
#define INA3221_POWER_VALID_LOWER (0x11)
#define INA3221_MANUFACTURER (0xFE)
#define INA3221_DIE_ID (0xFF)

#define MODE_VOLTAGE_CONTINUOUS 0x110        // Bus continuous
#define VOLTAGE_CONVERSION_TIME (0x11 << 6)  // 588us
//#define AVG 0x7     // 128 samples
//#define CH 0x7      // Enable all channels
#define RST 0x8000  // Reset bit

#define I2C_ADDRESS 0x40
#define SENSOR_INTERVAL_MS 20  // 10ms min for 1024 filter. 1ms min for 0B11 filter

static void begin(ina3221_parameters_t *parameter);
static void read(ina3221_parameters_t *parameter);

void ina3221_task(void *parameters) {
    ina3221_parameters_t parameter = *(ina3221_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    for (uint8_t i = 0; i < parameter.cell_count; i++) {
        *parameter.cell[i] = 0;
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);

    begin(&parameter);

    while (1) {
        read(&parameter);
        debug("\nINA3221 (%u) < Addr: 0x%02X", uxTaskGetStackHighWaterMark(NULL), parameter.i2c_address);
        for (uint8_t i = 0; i < parameter.cell_count; i++) {
            debug(" Cell %u: %.2fV", i + 1, *parameter.cell[i]);
        }
        vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void begin(ina3221_parameters_t *parameter) {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    uint8_t data[3] = {0};

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (i2c_write_blocking_until(i2c0, parameter->i2c_address, data, 1, false, make_timeout_time_ms(1000)) ==
           PICO_ERROR_GENERIC) {
        debug("\nINA3221 not found at address 0x%02X. Connect and reboot", parameter->i2c_address);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Configure sensor. INA3221 Configuration register is 16 bits wide: an I2C write
    // therefore needs register address + 2 data bytes (MSB first). Build the full
    // 16-bit value as uint16_t first, then split into high/low bytes — otherwise the
    // upper byte (which holds the channel-enable bits 12-14 and the AVG bits 9-11)
    // would be silently truncated by a single-byte store.
    if (parameter->cell_count > 3) parameter->cell_count = 3;
    if (parameter->cell_count < 1) parameter->cell_count = 1;
    uint16_t config = MODE_VOLTAGE_CONTINUOUS | VOLTAGE_CONVERSION_TIME |
                      ((uint16_t)parameter->filter << 9);
    for (uint8_t i = 1; i < parameter->cell_count; i++) {
        config |= (uint16_t)(1u << (i + 12));
    }
    data[0] = INA3221_CONFIGURATION;
    data[1] = (uint8_t)(config >> 8);    // MSB: CH enables, AVG, conversion time MSBs
    data[2] = (uint8_t)(config & 0xFF);  // LSB: conversion time LSBs, mode
    i2c_write_blocking(i2c0, parameter->i2c_address, data, 3, false);
}

static void read(ina3221_parameters_t *parameter) {
    uint8_t data[3];
    int16_t bus_voltage;

    float cell_total[3] = {0};
    for (uint8_t channel = 0; channel < parameter->cell_count; channel++) {
        // Read bus voltage
        data[0] = INA3221_BUS_VOLTAGE(channel);
        i2c_write_blocking(i2c0, parameter->i2c_address, data, 1, true);
        i2c_read_blocking(i2c0, parameter->i2c_address, data, 2, false);
        bus_voltage = ((int16_t)data[0] << 8) | data[1];
        // Calculate voltage in volts
        cell_total[channel] = (float)bus_voltage * 0.001f;  // Bus voltage LSB = 1mV
#ifdef SIM_SENSORS
        *parameter->cell[channel] = 3 + channel * 0.01;
#endif
    }
    for (uint8_t i = 0; i < parameter->cell_count; i++) {
        if (i == 0) {
            *parameter->cell[i] = cell_total[i] - *parameter->cell_prev;
        } else {
            *parameter->cell[i] = cell_total[i] - cell_total[i - 1];
        }
    }
}