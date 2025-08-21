#include "ads7830.h"

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "stdlib.h"
#include "cell_count.h"

#define ADDRESS 0b1001000
#define CONV_OFF (0 << 2)
#define CONV_ON (1 << 2)
#define REF_OFF (0 << 3)
#define REF_ON (1 << 3)
#define CH1 (0 << 4)
#define CH2 (1 << 4)
#define CH3 (2 << 4)
#define CH4 (3 << 4)
#define INVERTED (1 << 6)
#define NO_INVERTED (0 << 6)
#define DIFFERENTIAL_INPUTS (1 << 7)
#define SINGLE_ENDED_INPUTS (0 << 7)
#define CMD_CH1 (CONV_ON | REF_ON | CH1 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define CMD_CH2 (CONV_ON | REF_ON | CH2 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define CMD_CH3 (CONV_ON | REF_ON | CH3 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define CMD_CH4 (CONV_ON | REF_ON | CH4 | NO_INVERTED | DIFFERENTIAL_INPUTS)
#define SENSOR_INTERVAL_MS 10
#define VCC_VOLTAGE 3.3

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
    
    float voltage_total;
    TaskHandle_t task_handle;
    uint cell_count_delay = 15000;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, &voltage_total, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    uint8_t cmd[4] = {CMD_CH1, CMD_CH2, CMD_CH3, CMD_CH4};
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);
    while (1) {
        *parameter.cell[cont % 4] = read(&parameter, cmd[cont % 4]);
        voltage_total = *parameter.cell[0] + *parameter.cell[1] + *parameter.cell[2] + *parameter.cell[3];
        cont++;
        debug("\nADS78030 (%u) < Cell1 %u Cell2 %u Cell3 %u Cell3 %u", uxTaskGetStackHighWaterMark(NULL),
              *parameter.cell[0], *parameter.cell[1], *parameter.cell[2], *parameter.cell[3]);
    }
}

static float read(ads7830_parameters_t *parameters, uint8_t cmd) {
    ads7830_parameters_t parameter = *(ads7830_parameters_t *)parameters;
    uint8_t value;
    i2c_write_blocking(i2c0, parameter.address, &cmd, 1, false);
    i2c_read_blocking(i2c0, parameter.address, &value, 1, false);
    vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
#ifdef SIM_SENSORS
    value = 3.3;
#endif
    return (float)value / 0xFF * VCC_VOLTAGE;
}
