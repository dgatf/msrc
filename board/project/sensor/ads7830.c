#include "ads7830.h"

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "stdlib.h"

#define CONV_OFF (0 << 2)
#define CONV_ON (1 << 2)
#define REF_OFF (0 << 3)
#define REF_ON (1 << 3)
#define CH0 (0 << 4)          // 0-1
#define CH1 (1 << 4)          // 3-2
#define CH2 (2 << 4)          // 5-4
#define CH3 (3 << 4)          // 7-6
#define INVERTED (0 << 6)     // only for differential
#define NO_INVERTED (1 << 6)  // only for differential
#define DIFFERENTIAL_INPUTS (0 << 7)
#define SINGLE_ENDED_INPUTS (1 << 7)
#define CMD_CH0 (CONV_ON | REF_ON | CH0 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define CMD_CH1 (CONV_ON | REF_ON | CH1 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define CMD_CH2 (CONV_ON | REF_ON | CH2 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define CMD_CH3 (CONV_ON | REF_ON | CH3 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define SENSOR_INTERVAL_MS 1000
#define REF_VOLTAGE 5

static float read(ads7830_parameters_t *parameter, uint8_t cmd);

void ads7830_task(void *parameters) {
    ads7830_parameters_t parameter = *(ads7830_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    static uint8_t cont = 0;
    *parameter.cell[0] = 0;
    *parameter.cell[1] = 0;
    *parameter.cell[2] = 0;
    *parameter.cell[3] = 0;
    *parameter.cell_count = 1;
    uint8_t cmd[4] = {CMD_CH0, CMD_CH1, CMD_CH2, CMD_CH3};
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);
    while (1) {
        *parameter.cell[cont % 4] = read(&parameter, cmd[cont % 4]);
        uint cell_index = (cont % 4) + 1;  // 1 to 4
        if (*parameter.cell[cont % 4] > 1 && cell_index > *parameter.cell_count) *parameter.cell_count = cell_index;
        debug("\nADS7830 (%u). Cell index %u Cell count %u Cell1 %u Cell2 %u Cell3 %u Cell3 %u",
              uxTaskGetStackHighWaterMark(NULL), cell_index, *parameter.cell_count, *parameter.cell[0],
              *parameter.cell[1], *parameter.cell[2], *parameter.cell[3]);
        cont++;
        read(&parameter, cmd[cont % 4]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static float read(ads7830_parameters_t *parameters, uint8_t cmd) {
    ads7830_parameters_t parameter = *(ads7830_parameters_t *)parameters;
    uint8_t value = 0;
    i2c_write_blocking(i2c0, parameter.address, &cmd, 1, true);
    i2c_read_blocking(i2c0, parameter.address, &value, 1, true);
#ifdef SIM_SENSORS
    value = 3.3;
#endif
    float volt = (float)value / 0xFF * REF_VOLTAGE;
    if (volt < 1) volt = 0;
    return volt;
}
