#ifndef BMP280_H
#define BMP280_H


#include <Arduino.h>
#include "device.h"
#include "i2c.h"

#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA

#define BMP280_OVERSAMPLING_X0 0
#define BMP280_OVERSAMPLING_X1 1
#define BMP280_OVERSAMPLING_X2 2
#define BMP280_OVERSAMPLING_X4 3
#define BMP280_OVERSAMPLING_X8 4
#define BMP280_OVERSAMPLING_X16 5

#define BMP280_SLEEP 0
#define BMP280_FORCED 1
#define BMP280_NORMAL 3

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

#define BMP280_VARIO_INTERVAL 500
#define BMP280_MEASUREMENT_INTERVAL 30

class Bmp280 : public AbstractDevice, I2C, Vario
{
private:
    uint16_t T1_, P1_;
    int16_t T2_, T3_, P2_, P3_, P4_, P5_, P6_, P7_, P8_, P9_;
    uint32_t t_fine_;
    float temperature_ = 0, pressure_ = 0, P0_ = 0, altitude_ = 0, vario_;
    uint8_t device_, alphaTemp_, alphaDef_;
    void readTemperature();
    void readPressure();
    void calcAltitude();

public:
    Bmp280(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef);
    void begin();
    void update();
    float *temperatureP();
    float *pressureP();
    float *altitudeP();
    float *varioP();
};

#endif