#include "bmp280.h"

Bmp280::Bmp280(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef) : device_(device), alphaTemp_(alphaTemp), alphaDef_(alphaDef) {}

void Bmp280::begin()
{
    uint8_t configReg[1] = {(STANDBY_MS_250 << 5) | (FILTER_X8 << 2) | 0};
    uint8_t measureReg[1] = {(BMP280_OVERSAMPLING_X4 << 5) | (BMP280_OVERSAMPLING_X4 << 2) | BMP280_NORMAL};
    writeBytes(device_, BMP280_REGISTER_CONFIG, configReg, 1);
    writeBytes(device_, BMP280_REGISTER_CONTROL, measureReg, 1);
    T1_ = readUInt(device_, 0x88, I2C_LITTLE_ENDIAN);
    T2_ = readInt(device_, 0x8A, I2C_LITTLE_ENDIAN);
    T3_ = readInt(device_, 0x8C, I2C_LITTLE_ENDIAN);
    P1_ = readUInt(device_, 0x8E, I2C_LITTLE_ENDIAN);
    P2_ = readInt(device_, 0x90, I2C_LITTLE_ENDIAN);
    P3_ = readInt(device_, 0x92, I2C_LITTLE_ENDIAN);
    P4_ = readInt(device_, 0x94, I2C_LITTLE_ENDIAN);
    P5_ = readInt(device_, 0x96, I2C_LITTLE_ENDIAN);
    P6_ = readInt(device_, 0x98, I2C_LITTLE_ENDIAN);
    P7_ = readInt(device_, 0x9A, I2C_LITTLE_ENDIAN);
    P8_ = readInt(device_, 0x9C, I2C_LITTLE_ENDIAN);
    P9_ = readInt(device_, 0x9E, I2C_LITTLE_ENDIAN);
}

void Bmp280::readTemperature()
{
    uint8_t data[3];
    readBytes(device_, BMP280_REGISTER_TEMPDATA, data, 3);
    int32_t var1, var2, t;
    int32_t adc_T = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2]) >> 4;
    var1 = ((((adc_T >> 3) - ((int32_t)T1_ << 1))) * ((int32_t)T2_)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)T1_)) * ((adc_T >> 4) - ((int32_t)T1_))) >> 12) * ((int32_t)T3_)) >> 14;
    t_fine_ = var1 + var2;
    t = (t_fine_ * 5 + 128) >> 8;
    temperature_ = (float)t / 100;
    //temperature_ = calcAverage((float)alphaTemp_ / 100, temperature_, (float)t / 100);
}

void Bmp280::readPressure()
{
    int64_t var1, var2, p;
    uint8_t data[3];
    uint32_t adc_P;
    readTemperature();
    readBytes(device_, BMP280_REGISTER_PRESSUREDATA, data, 3);
    adc_P = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2]) >> 4;
    var1 = ((int64_t)t_fine_) - 128000;
    var2 = var1 * var1 * (int64_t)P6_;
    var2 = var2 + ((var1 * (int64_t)P5_) << 17);
    var2 = var2 + (((int64_t)P4_) << 35);
    var1 = ((var1 * var1 * (int64_t)P3_) >> 8) +
           ((var1 * (int64_t)P2_) << 12);
    var1 =
        (((((int64_t)1) << 47) + var1)) * ((int64_t)P1_) >> 33;

    if (var1 != 0)
    {
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)P9_) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)P8_) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)P7_) << 4);
        pressure_ = (float)p / 256;
        //pressure_ = calcAverage((float)alphaDef_ / 100, pressure_, (float)p / 256);
    }
}

float Bmp280::calcAltitude(float pressure)
{
    if (P0_ == 0 && millis() > 5000)
#ifdef SIM_SENSORS
        P0 = 600;
#else
        P0_ = pressure_;
#endif
    if (P0_ == 0)
        return 0;
    return 44330.0 * (1 - pow(pressure_ / P0_, 1 / 5.255));
}

void Bmp280::update()
{
#ifdef SIM_SENSORS
    temperature_ = 20;
    pressure_ = 500;
#else
    readPressure();
#endif
    altitude_ = calcAltitude(pressure_);
    if (altitude_ < 0)
        altitude_ = 0;
}

float *Bmp280::temperatureP()
{
    return &temperature_;
}

float *Bmp280::pressureP()
{
    return &pressure_;
}

float *Bmp280::altitudeP()
{
    return &altitude_;
}