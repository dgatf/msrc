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
    for (uint8_t i = 0; i < 12; i++) {
        *parameter.cell[i] = 0;
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);

    begin(&parameter);

    while (1) {
        read(&parameter);
        debug("\nINA3221 (%u) < Addr: 0x%02X", uxTaskGetStackHighWaterMark(NULL), parameter.i2c_address[0]);
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

    uint8_t data[2] = {0};

    // Find sensor address
    parameter->i2c_address[0] = I2C_ADDRESS;
    for (uint8_t i = 1; i < 4; i++) {
        if (i2c_write_blocking(i2c0, I2C_ADDRESS, data, 1, false) == PICO_ERROR_GENERIC) {
            parameter->i2c_address[i] = I2C_ADDRESS + i;
        }
    }

    // Configure sensor
    data[0] = INA3221_CONFIGURATION;
    data[1] = MODE_VOLTAGE_CONTINUOUS | VOLTAGE_CONVERSION_TIME | (parameter->filter << 9);
    if (parameter->cell_count > 3) parameter->cell_count = 3;
    if (parameter->cell_count < 1) parameter->cell_count = 1;
    for (uint8_t i = 1; i < parameter->cell_count; i++) {
        data[1] |= (1 << (i + 12));
    }
    i2c_write_blocking(i2c0, parameter->i2c_address[0], data, 2, false);
}

static void read(ina3221_parameters_t *parameter) {
    uint8_t data[3];
    int16_t bus_voltage;

    for (uint8_t channel = 0; channel < 3; channel++) {
        // Read bus voltage
        data[0] = INA3221_BUS_VOLTAGE(channel);
        i2c_write_blocking(i2c0, parameter->i2c_address[0], data, 1, true);
        i2c_read_blocking(i2c0, parameter->i2c_address[0], data, 2, false);
        bus_voltage = ((int16_t)data[0] << 8) | data[1];
        // Calculate voltage in volts
        *parameter->cell[channel] = (float)bus_voltage * 0.001f * 3.3 / 5;  // Bus voltage LSB = 1mV
#ifdef SIM_SENSORS
        *parameter->cell[channel] = 3 + channel * 0.01;
#endif
    }
}