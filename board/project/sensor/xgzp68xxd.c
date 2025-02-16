#include "xgzp68xxd.h"

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "stdlib.h"

#define I2C_ADDRESS 0X6D

#define REG_CMD 0x30
#define REG_DATA 0X06

#define CONTROL_SINGLE 0b10
#define CONTROL_CONT 0b11

#define SLEEP_TIME_62 0b0001   // 62.5ms
#define SLEEP_TIME_125 0b0010  // 125 ms
#define SLEEP_TIME_187 0b0011  // 187.5ms

#define SENSOR_INTERVAL_MS 100  // min 62.5ms

static void read(xgzp68xxd_parameters_t *parameter);
static void begin(void);

void xgzp68xxd_task(void *parameters) {
    xgzp68xxd_parameters_t parameter = *(xgzp68xxd_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    *parameter.temperature = 0;
    *parameter.pressure = 0;
    TaskHandle_t task_handle;

    vTaskDelay(500 / portTICK_PERIOD_MS);
    begin();
    while (1) {
        read(&parameter);
        debug("\nXGZP68XXD (%u) < Temp(C): %.2f Pressure(kPa): %.2f", uxTaskGetStackHighWaterMark(NULL),
              *parameter.temperature, *parameter.pressure / 1000);
        vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void read(xgzp68xxd_parameters_t *parameter) {
    int16_t temperature_raw;
    int32_t pressure_raw;
    uint8_t data[5];
    uint8_t reg[1] = {REG_DATA};

    i2c_write_blocking(i2c0, I2C_ADDRESS, reg, 1, true);
    i2c_read_blocking(i2c0, I2C_ADDRESS, data, 5, false);

    pressure_raw = (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8)) >> 8;
    temperature_raw = ((uint16_t)data[3] << 8) | data[4];
    *parameter->temperature = temperature_raw / 256.0;          // C
    *parameter->pressure = (float)pressure_raw / parameter->k;  // Pa
#ifdef SIM_SENSORS
    *parameter->temperature = 12.34;  // C
    *parameter->pressure = 101325;    // Pa
#endif
}

static void begin(void) {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    // Set continuous mode at 62.5ms interval
    uint8_t data[2];
    data[0] = REG_CMD;
    data[1] = SLEEP_TIME_62;
    i2c_write_blocking(i2c0, I2C_ADDRESS, data, 2, false);

    // Wait for first reading
    vTaskDelay(20 / portTICK_PERIOD_MS);
}
