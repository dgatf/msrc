#include "ms5611.h"

static void read(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration);
static void begin(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration);

void ms5611_task(void *parameters)
{
    ms5611_parameters_t parameter = *(ms5611_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    *parameter.altitude = 0;
    *parameter.vspeed = 0;
    *parameter.temperature = 0;
    *parameter.pressure = 0;
    
    TaskHandle_t task_handle;
    
    /*if (parameter.auto_offset)
    {
        float pressure_offset = 0;
        uint pressure_offset_delay = 15000;
        auto_offset_parameters_t pressure_offset_parameters = {pressure_offset_delay, parameter.pressure, &pressure_offset};
        xTaskCreate(auto_offset_task, "ms5611_pressure_offset_task", STACK_AUTO_OFFSET, (void *)&pressure_offset_parameters, 1, &task_handle);
    }*/

    uint vspeed_interval = 500;
    vspeed_parameters_t parameters_vspeed = {vspeed_interval, parameter.altitude, parameter.vspeed};
    xTaskCreate(vspeed_task, "vspeed_task", STACK_VSPEED, (void *)&parameters_vspeed, 2, &task_handle);
    xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    ms5611_calibration_t calibration;
    begin(&parameter, &calibration);
    while (1)
    {
        read(&parameter, &calibration);
        if (debug)
            printf("\nMS5611 (%u) < Temp: %.2f Pressure: %.0f Altitude: %0.2f Vspeed: %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.temperature, *parameter.pressure, *parameter.altitude, *parameter.vspeed);
    }
}

static void read(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration)
{
    static float pressure_initial = 0;
    static uint discard_readings = 5;
    /* Read sensor data */
    uint32_t D1, D2;
    uint8_t data[3];
    data[0] = MS5611_CMD_CONV_D1 + MS5611_OVERSAMPLING_4096;
    i2c_write_blocking(i2c0, parameter->address, data, 1, false);
    vTaskDelay(MS5611_SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = MS5611_CMD_ADC_READ;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    D1 = (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
    data[0] = MS5611_CMD_CONV_D2 + MS5611_OVERSAMPLING_4096;
    i2c_write_blocking(i2c0, parameter->address, data, 1, false);
    vTaskDelay(MS5611_SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = MS5611_CMD_ADC_READ;
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

    if (TEMP < 2000)
    {
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;
    }
    if (TEMP < -1500)
    {
        OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
        SENS2 = SENS2 + 11 * (TEMP + 1500) * (TEMP + 1500) / 2;
    }
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;
    *parameter->temperature = (float)TEMP / 100; // °C
    *parameter->pressure = (float)P;             // Pa
    if (pressure_initial == 0 && discard_readings == 0)
        pressure_initial = *parameter->pressure;
    *parameter->altitude = get_altitude(*parameter->pressure, *parameter->temperature, pressure_initial);
    if (discard_readings > 0)
        discard_readings--;
    if (debug)
        printf("\nMS5611 P0: %.0f", pressure_initial);
#ifdef SIM_SENSORS
    *parameter->temperature = 12.34;
    *parameter->pressure = 1234.56;
    *parameter->altitude = 2345;
#endif
}

static void begin(ms5611_parameters_t *parameter, ms5611_calibration_t *calibration)
{
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    uint8_t data[12];
    data[0] = MS5611_CMD_RESET;
    i2c_write_blocking(i2c0, parameter->address, data, 1, false);
    vTaskDelay(MS5611_SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = MS5611_CMD_READ_PROM + 2;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C1 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = MS5611_CMD_READ_PROM + 4;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C2 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = MS5611_CMD_READ_PROM + 6;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C3 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = MS5611_CMD_READ_PROM + 8;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C4 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = MS5611_CMD_READ_PROM + 10;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C5 = ((uint16_t)data[0] << 8) | data[1];
    data[0] = MS5611_CMD_READ_PROM + 12;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    calibration->C6 = ((uint16_t)data[0] << 8) | data[1];
}
