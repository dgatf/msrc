#include "bmp180.h"

static void read(bmp180_parameters_t *parameter, bmp180_calibration_t *calibration);
static void begin(bmp180_parameters_t *parameter, bmp180_calibration_t *calibration);

void bmp180_task(void *parameters)
{
    bmp180_parameters_t parameter = *(bmp180_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    *parameter.altitude = 0;
    *parameter.vspeed = 0;
    *parameter.temperature = 0;
    *parameter.pressure = 0;

    TaskHandle_t task_handle;

    /*if (parameter.auto_offset)
    {
        uint pressure_offset_delay = 15000;
        auto_offset_parameters_t pressure_offset_parameters = {pressure_offset_delay, parameter.pressure, &pressure_offset};
        xTaskCreate(auto_offset_task, "bmp180_pressure_offset_task", STACK_AUTO_OFFSET, (void *)&pressure_offset_parameters, 1, &task_handle);
    }*/

    uint vspeed_interval = 500;
    vspeed_parameters_t parameters_vspeed = {vspeed_interval, parameter.altitude, parameter.vspeed};
    xTaskCreate(vspeed_task, "vspeed_task", STACK_VSPEED, (void *)&parameters_vspeed, 2, &task_handle);
    xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    bmp180_calibration_t calibration;
    begin(&parameter, &calibration);
    while (1)
    {
        read(&parameter, &calibration);
        if (debug)
            printf("\nBMP180 (%u) < Temp: %.2f Pressure: %.0f Altitude: %.2f Vspeed: %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.temperature, *parameter.pressure, *parameter.altitude, *parameter.vspeed);
    }
}

static void read(bmp180_parameters_t *parameter, bmp180_calibration_t *calibration)
{
    uint8_t register_address, register_value;
    uint8_t data[3];
    int32_t X1, X2, X3, B5, T, UT, UP, B6, B3, p;
    uint32_t B4, B7;
    static float pressure_initial = 0;
    static uint discard_readings = 5;

    data[0] = BMP180_REGISTER_CONTROL;
    data[1] = BMP180_READ_TEMPERATURE;
    i2c_write_blocking(i2c0, parameter->address, data, 2, false);
    vTaskDelay(BMP180_TEMPERATURE_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = BMP180_REGISTER_DATA;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 2, false);
    UT = data[0] << 8 | data[1];
    X1 = ((UT - calibration->AC6) * calibration->AC5) >> 15;
    X2 = (calibration->MC << 11) / (X1 + calibration->MD);
    B5 = X1 + X2;
    T = (B5 + 8) >> 4;
    *parameter->temperature = (float)T / 10; // C     calcAverage((float)alphaTemp_ / 100, temperature_, (float)T / 10);

    data[0] = BMP180_REGISTER_CONTROL;
    data[1] = BMP180_READ_PRESSURE | (BMP180_OVERSAMPLING_3 << 6);
    i2c_write_blocking(i2c0, parameter->address, data, 2, false);
    vTaskDelay(BMP180_PRESSURE_INTERVAL_MS / portTICK_PERIOD_MS);
    data[0] = BMP180_REGISTER_DATA;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    UP = ((uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2]) >> (8 - BMP180_OVERSAMPLING_3);
    B6 = B5 - 4000;
    X1 = (calibration->B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (calibration->AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((calibration->AC1 * 4 + X3) << BMP180_OVERSAMPLING_3) + 2) / 4;
    X1 = (calibration->AC3 * B6) >> 13;
    X2 = (calibration->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = calibration->AC4 * (uint32_t)(X3 + 32768) >> 15;
    B7 = (uint32_t)(UP - B3) * (50000 >> BMP180_OVERSAMPLING_3);
    if (B7 < 0x80000000)
    {
        p = B7 * 2 / B4;
    }
    else
    {
        p = B7 * B4 / 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);

    *parameter->pressure = p; // Pa    calcAverage((float)alphaVario_ / 100, pressure_, p);
    if (pressure_initial == 0 && discard_readings == 0)
        pressure_initial = *parameter->pressure;
    *parameter->altitude = get_altitude(*parameter->pressure, *parameter->temperature, pressure_initial);
    if (discard_readings > 0)
        discard_readings--;
    if (debug)
        printf("\nBMP180 P0: %.0f", pressure_initial);
#ifdef SIM_SENSORS
    *parameter->temperature = 12.34;
    *parameter->pressure = 1234.56;
    *parameter->altitude = 2345;
#endif
}

static void begin(bmp180_parameters_t *parameter, bmp180_calibration_t *calibration)
{
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    uint8_t data[22];
    data[0] = BMP180_REGISTER_DIG_AC1;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 22, false);
    calibration->AC1 = ((uint16_t)data[0] << 8) | data[1];
    calibration->AC2 = ((uint16_t)data[2] << 8) | data[3];
    calibration->AC3 = ((uint16_t)data[4] << 8) | data[5];
    calibration->AC4 = ((uint16_t)data[6] << 8) | data[7];
    calibration->AC5 = ((uint16_t)data[8] << 8) | data[9];
    calibration->AC6 = ((uint16_t)data[10] << 8) | data[11];
    calibration->B1 = ((uint16_t)data[12] << 8) | data[13];
    calibration->B2 = ((uint16_t)data[14] << 8) | data[15];
    calibration->MB = ((uint16_t)data[16] << 8) | data[17];
    calibration->MC = ((uint16_t)data[18] << 8) | data[19];
    calibration->MD = ((uint16_t)data[20] << 8) | data[21];
}
