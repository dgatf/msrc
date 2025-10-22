#include "bmp280.h"

#include <stdio.h>

#include "auto_offset.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "vspeed.h"

#define REGISTER_DIG_T1 0x88
#define REGISTER_DIG_T2 0x8A
#define REGISTER_DIG_T3 0x8C
#define REGISTER_DIG_P1 0x8E
#define REGISTER_DIG_P2 0x90
#define REGISTER_DIG_P3 0x92
#define REGISTER_DIG_P4 0x94
#define REGISTER_DIG_P5 0x96
#define REGISTER_DIG_P6 0x98
#define REGISTER_DIG_P7 0x9A
#define REGISTER_DIG_P8 0x9C
#define REGISTER_DIG_P9 0x9E
#define REGISTER_CHIPID 0xD0
#define REGISTER_VERSION 0xD1
#define REGISTER_SOFTRESET 0xE0
#define REGISTER_CAL26 0xE1
#define REGISTER_STATUS 0xF3
#define REGISTER_CONTROL 0xF4
#define REGISTER_CONFIG 0xF5
#define REGISTER_PRESSUREDATA 0xF7
#define REGISTER_TEMPDATA 0xFA

#define OVERSAMPLING_X0 0
#define OVERSAMPLING_X1 1
#define OVERSAMPLING_X2 2
#define OVERSAMPLING_X4 3
#define OVERSAMPLING_X8 4
#define OVERSAMPLING_X16 5

#define SLEEP 0
#define FORCED 1
#define NORMAL 3

#define FILTER_OFF 0
#define FILTER_X2 1
#define FILTER_X4 2
#define FILTER_X8 3
#define FILTER_X16 4

#define STANDBY_MS_1 0x00
#define STANDBY_MS_63 0x01
#define STANDBY_MS_125 0x02
#define STANDBY_MS_250 0x03
#define STANDBY_MS_500 0x04
#define STANDBY_MS_1000 0x05
#define STANDBY_MS_2000 0x06
#define STANDBY_MS_4000 0x07

#define I2C_ADDRESS_1 0x76
#define I2C_ADDRESS_2 0x77
#define SENSOR_INTERVAL_MS 40  // min 30
#define VSPEED_INTERVAL_MS 500

static void read(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration);
static void begin(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration);

void bmp280_task(void *parameters) {
    bmp280_parameters_t parameter = *(bmp280_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    *parameter.altitude = 0;
    *parameter.vspeed = 0;
    *parameter.temperature = 0;
    *parameter.pressure = 0;

    TaskHandle_t task_handle;

    bmp280_calibration_t calibration;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    begin(&parameter, &calibration);
    while (1) {
        read(&parameter, &calibration);
        debug("\nBMP280 (%u) < Temp: %.2f Pressure: %.0f Altitude: %0.2f Vspeed: %.2f",
              uxTaskGetStackHighWaterMark(NULL), *parameter.temperature, *parameter.pressure, *parameter.altitude,
              *parameter.vspeed);
        vTaskDelay(SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void read(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration) {
    int64_t var1, var2, p;
    uint8_t data[3];
    uint32_t adc_T, adc_P, t_fine, t;
    static float pressure_initial = 0;
    static uint discard_readings = 5;

    data[0] = REGISTER_TEMPDATA;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    adc_T = (((uint32_t)data[0] << 16) | ((uint16_t)data[1] << 8) | data[2]) >> 4;
    var1 = ((((adc_T >> 3) - ((int32_t)calibration->T1 << 1))) * ((int32_t)calibration->T2)) >> 11;
    var2 = (int64_t)(((((adc_T >> 4) - ((int32_t)calibration->T1)) * ((adc_T >> 4) - ((int32_t)calibration->T1))))) *
               (calibration->T3) >>
           26;
    t_fine = var1 + var2;
    t = (t_fine * 5 + 128) >> 8;
    *parameter->temperature = (float)t / 100;

    data[0] = REGISTER_PRESSUREDATA;
    i2c_write_blocking(i2c0, parameter->address, data, 1, true);
    i2c_read_blocking(i2c0, parameter->address, data, 3, false);
    adc_P = (((uint32_t)data[0] << 16) | ((uint16_t)data[1] << 8) | data[2]) >> 4;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration->P6;
    var2 = var2 + ((var1 * (int64_t)calibration->P5) << 17);
    var2 = var2 + (((int64_t)calibration->P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calibration->P3) >> 8) + ((var1 * (int64_t)calibration->P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration->P1) >> 33;

    if (var1 != 0) {
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)calibration->P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)calibration->P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)calibration->P7) << 4);
        *parameter->pressure = (float)p / 256;  // Pa
        // pressure_ = calcAverage((float)alphaVario_ / 100, pressure_, (float)p / 256);
    }

    if (pressure_initial == 0 && discard_readings == 0) pressure_initial = *parameter->pressure;
    *parameter->altitude = get_altitude(*parameter->pressure, *parameter->temperature, pressure_initial);
    get_vspeed(parameter->vspeed, *parameter->altitude, VSPEED_INTERVAL_MS);
    if (discard_readings > 0) discard_readings--;
    debug("\nBMP280 P0: %.0f", pressure_initial);
#ifdef SIM_SENSORS
    *parameter->temperature = 12.34;
    *parameter->pressure = 1234.56;
    *parameter->altitude = 2345;
#endif
}

static void begin(bmp280_parameters_t *parameter, bmp280_calibration_t *calibration) {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    uint8_t data[24] = {0};

    // Find sensor address
    parameter->address = I2C_ADDRESS_1;
    if (i2c_write_blocking(i2c0, I2C_ADDRESS_1, data, 1, false) == PICO_ERROR_GENERIC) {
        parameter->address = I2C_ADDRESS_2;
    }

    // Configure sensor
    data[0] = REGISTER_CONTROL;
    data[1] = (OVERSAMPLING_X2 << 5) | (OVERSAMPLING_X16 << 2) | NORMAL;
    i2c_write_blocking(i2c0, parameter->address, data, 2, false);

    data[0] = REGISTER_DIG_T1;
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
