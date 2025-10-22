#include "ms5611.h"

#include <stdio.h>

#include "auto_offset.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "stdlib.h"
#include "vspeed.h"

#define CMD_ADC_READ 0x00
#define CMD_RESET 0x1E
#define CMD_CONV_D1 0x40
#define CMD_CONV_D2 0x50

// 0 0xA0,0xA1 - reserved
// 1 0xA2,0xA3 - C1
// 2 0xA4,0xA5 - C2
// 3 0xA6,0xA7 - C3
// 4 0xA8,0xA9 - C4
// 5 0xAA,0xAB - C5
// 6 0xAC,0xAD - C6
// 7 0xAE - CRC (4bits)

#define CMD_READ_PROM 0xA0

#define OVERSAMPLING_4096 0x08
#define OVERSAMPLING_2048 0x06
#define OVERSAMPLING_1024 0x04
#define OVERSAMPLING_512 0x02
#define OVERSAMPLING_256 0x00

#define I2C_ADDRESS_1 0x77
#define I2C_ADDRESS_2 0x76

#define SENSOR_INTERVAL_MS 20  // min 10
#define VSPEED_INTERVAL_MS 500

static void read(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration);
static void begin(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration);

void ms5611_task(void *parameters) {
    ms5611_parameters_t parameter = *(ms5611_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    *parameter.altitude = 0;
    *parameter.vspeed = 0;
    *parameter.temperature = 0;
    *parameter.pressure = 0;

    TaskHandle_t task_handle;

    vTaskDelay(500 / portTICK_PERIOD_MS);
    ms5611_calibration_t calibration;
    begin(&parameter, &calibration);
    while (1) {
        read(&parameter, &calibration);
        debug("\nMS5611 (%u) < Temp: %.2f Pressure: %.0f Altitude: %0.2f Vspeed: %.2f",
              uxTaskGetStackHighWaterMark(NULL), *parameter.temperature, *parameter.pressure, *parameter.altitude,
              *parameter.vspeed);
    }
}

static void read(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration) {
    static float pressure_initial = 0;
    static uint discard_readings = 5;
    /* Read sensor data */
    uint32_t D1, D2;
    uint8_t data[3];
    data[0] = CMD_CONV_D1 + OVERSAMPLING_4096;
    i2c_write_blocking(i2c0, parameter->address, data, 1, false);
    vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = CMD_ADC_READ;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    D1 = (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
    data[0] = CMD_CONV_D2 + OVERSAMPLING_4096;
    i2c_write_blocking(i2c0, parameter->address, data, 1, false);
    vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = CMD_ADC_READ;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    D2 = (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];

    /* Calculation */
    int32_t dT, TEMP, T2 = 0;
    int64_t OFF, SENS, OFF2 = 0, SENS2 = 0;
    dT = D2 - ((uint32_t)calibration->C5 << 8);
    TEMP = 2000 + ((int64_t)dT * calibration->C6 >> 23);
    OFF = ((int64_t)calibration->C2 << 16) + (((int64_t)calibration->C4 * dT) >> 7);
    SENS = ((int64_t)calibration->C1 << 15) + (((int64_t)calibration->C3 * dT) >> 8);

    if (TEMP < 2000) {
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;
    }
    if (TEMP < -1500) {
        OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
        SENS2 = SENS2 + 11 * (TEMP + 1500) * (TEMP + 1500) / 2;
    }
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;
    *parameter->temperature = (float)TEMP / 100;  // °C
    *parameter->pressure = (float)P;              // Pa
    if (pressure_initial == 0 && discard_readings == 0) pressure_initial = *parameter->pressure;
    *parameter->altitude = get_altitude(*parameter->pressure, *parameter->temperature, pressure_initial);
    get_vspeed(parameter->vspeed, *parameter->altitude, VSPEED_INTERVAL_MS);
    if (discard_readings > 0) discard_readings--;
    debug("\nMS5611 P0: %.0f", pressure_initial);
#ifdef SIM_SENSORS
    *parameter->temperature = 12.34;
    *parameter->pressure = 1234.56;
    *parameter->altitude = 2345;
#endif
}

static void begin(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration) {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    uint8_t data[12];

    // Find sensor address
    parameter->address = I2C_ADDRESS_1;
    if (i2c_write_blocking(i2c0, I2C_ADDRESS_1, data, 1, false) == PICO_ERROR_GENERIC) {
        parameter->address = I2C_ADDRESS_2;
    }

    // Read calibration data
    data[0] = CMD_RESET;
    i2c_write_blocking(i2c0, parameter->address, data, 1, false);
    vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = CMD_READ_PROM + 2;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C1 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = CMD_READ_PROM + 4;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C2 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = CMD_READ_PROM + 6;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C3 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = CMD_READ_PROM + 8;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C4 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = CMD_READ_PROM + 10;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C5 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = CMD_READ_PROM + 12;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C6 = ((uint16_t)data[0] << 8) | data[1];
}
