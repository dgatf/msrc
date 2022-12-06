#include "bmp280.h"

static void read(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration, float *pressure_offset);
static void begin(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration);

void bmp280_task(void *parameters)
{
    bmp280_parameters_t parameter = *(bmp280_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    *parameter.altitude = 0;
    *parameter.vspeed = 0;
    *parameter.temperature = 0;
    *parameter.pressure = 0;

    TaskHandle_t task_handle;
    float pressure_offset = 0;
    if (parameter.auto_offset)
    {
        float pressure_offset = 0;
        uint pressure_offset_delay = 15000;
        auto_offset_parameters_t pressure_offset_parameters = {pressure_offset_delay, parameter.pressure, &pressure_offset};
        xTaskCreate(auto_offset_task, "bmp280_pressure_offset_task", STACK_AUTO_OFFSET, (void *)&pressure_offset_parameters, 1, &task_handle);
    }

    uint vspeed_interval = 500;
    vspeed_parameters_t parameters_vspeed = {vspeed_interval, parameter.altitude, parameter.vspeed};
    xTaskCreate(vspeed_task, "vspeed_task", STACK_VSPEED, (void *)&parameters_vspeed, 2, &task_handle);
    xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    bmp280_calibration_t calibration;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    begin(&parameter, &calibration);
    while (1)
    {
        read(&parameter, &calibration, &pressure_offset);
        if (debug)
        {
            printf("\nBMP280 (%u) < Temp: %.2f Pressure: %.0f Altitude: %0.2f Vspeed: %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.temperature, *parameter.pressure, *parameter.altitude, *parameter.vspeed);
        }
        vTaskDelay(BMP280_SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void read(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration, float *pressure_offset)
{
    int64_t var1, var2, p;
    uint8_t data[3];
    uint32_t adc_T, adc_P, t_fine, t;

    data[0] = BMP280_REGISTER_TEMPDATA;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    adc_T = (((uint32_t)data[0] << 16) | ((uint16_t)data[1] << 8) | data[2]) >> 4;
    var1 = ((((adc_T >> 3) - ((int32_t)calibration->T1 << 1))) * ((int32_t)calibration->T2)) >> 11;
    var2 = (int64_t)(((((adc_T >> 4) - ((int32_t)calibration->T1)) * ((adc_T >> 4) - ((int32_t)calibration->T1))))) * (calibration->T3) >> 26;
    t_fine = var1 + var2;
    t = (t_fine * 5 + 128) >> 8;
    *parameter->temperature = (float)t / 100;

    data[0] = BMP280_REGISTER_PRESSUREDATA;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    adc_P = (((uint32_t)data[0] << 16) | ((uint16_t)data[1] << 8) | data[2]) >> 4;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration->P6;
    var2 = var2 + ((var1 * (int64_t)calibration->P5) << 17);
    var2 = var2 + (((int64_t)calibration->P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calibration->P3) >> 8) +
           ((var1 * (int64_t)calibration->P2) << 12);
    var1 =
        (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration->P1) >> 33;

    if (var1 != 0)
    {
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)calibration->P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)calibration->P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)calibration->P7) << 4);
        *parameter->pressure = (float)p / 256; // Pa
        // pressure_ = calcAverage((float)alphaVario_ / 100, pressure_, (float)p / 256);
    }
    *parameter->altitude = get_altitude(*parameter->pressure, *parameter->temperature, *pressure_offset);
#ifdef SIM_SENSORS
    *parameter->temperature = 12.34;
    *parameter->pressure = 1234.56;
    *parameter->altitude = 2345;
#endif
}

static void begin(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration)
{
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    uint8_t data[24] = {0};
    data[0] = BMP280_REGISTER_CONFIG;
    data[1] = (STANDBY_MS_1 << 5) | ((BMP280_FILTER + 1) << 2) | 0;
    i2c_write_blocking(i2c0, parameter->address, data, 2, false);

    data[0] = BMP280_REGISTER_CONTROL;
    data[1] = (BMP280_OVERSAMPLING_X2 << 5) | (BMP280_OVERSAMPLING_X16 << 2) | BMP280_NORMAL;
    i2c_write_blocking(i2c0, parameter->address, data, 2, false);

    data[0] = BMP280_REGISTER_DIG_T1;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 24, false);
    calibration->T1 = ((uint16_t)data[1] << 8) | data[0];
    calibration->T2 = ((uint16_t)data[3] << 8) | data[2];
    calibration->T3 = ((uint16_t)data[5] << 8) | data[4];
    calibration->P1 = ((uint16_t)data[7] << 8) | data[6];
    calibration->P2 = ((uint16_t)data[9] << 8) | data[8];
    calibration->P3 = ((uint16_t)data[11] << 8) | data[10];
    calibration->P4 = ((uint16_t)data[13] << 8) | data[12];
    calibration->P5 = ((uint16_t)data[15] << 8) | data[14];
    calibration->P6 = ((uint16_t)data[17] << 8) | data[16];
    calibration->P7 = ((uint16_t)data[19] << 8) | data[18];
    calibration->P8 = ((uint16_t)data[21] << 8) | data[20];
    calibration->P9 = ((uint16_t)data[23] << 8) | data[22];
}
