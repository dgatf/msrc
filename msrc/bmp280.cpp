#include "bmp280.h"

Bmp280Interface::Bmp280Interface(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef) : Bmp(device, alphaTemp, alphaDef) {}

bool Bmp280Interface::begin()
{
    uint8_t configReg[1] = {(STANDBY_MS_250 << 5) | (FILTER_X8 << 2) | 0};
    uint8_t measureReg[1] = {(BMP280_OVERSAMPLING_X4 << 5) | (BMP280_OVERSAMPLING_X4 << 2) | BMP280_NORMAL};
    writeBytes(device_, BMP280_REGISTER_CONFIG, configReg, 1);
    writeBytes(device_, BMP280_REGISTER_CONTROL, measureReg, 1);
    T1_ = readUInt(device_, 0x88, LITTLE_ENDIAN); 
    T2_ = readInt(device_, 0x8A, LITTLE_ENDIAN);
    T3_ = readInt(device_, 0x8C, LITTLE_ENDIAN);
    P1_ = readUInt(device_, 0x8E, LITTLE_ENDIAN);
    P2_ = readInt(device_, 0x90, LITTLE_ENDIAN);
    P3_ = readInt(device_, 0x92, LITTLE_ENDIAN);
    P4_ = readInt(device_, 0x94, LITTLE_ENDIAN);
    P5_ = readInt(device_, 0x96, LITTLE_ENDIAN);
    P6_ = readInt(device_, 0x98, LITTLE_ENDIAN);
    P7_ = readInt(device_, 0x9A, LITTLE_ENDIAN);
    P8_ = readInt(device_, 0x9C, LITTLE_ENDIAN);
    P9_ = readInt(device_, 0x9E, LITTLE_ENDIAN);
    return true;
}

float Bmp280Interface::readTemperature()
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
    return temperature_;
}

float Bmp280Interface::readPressure()
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

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)P9_) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)P8_) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)P7_) << 4);
    pressure_ = (float)p / 256;
    //pressure_ = calcAverage((float)alphaDef_ / 100, pressure_, (float)p / 256);
    return pressure_;
}

float Bmp280Interface::read(uint8_t index)
{
    if (index == BMP_TEMPERATURE)
    {
#ifdef SIM_SENSORS
        return 22;
#endif
        readTemperature();
        return temperature_;
    }
    if (index == BMP_ALTITUDE) {
#ifdef SIM_SENSORS
        return 500;
#endif
        if (readPressure())
        {
            altitude_ = calcAltitude();
            if (altitude_ < 0 ) altitude_ = 0;
        }
        return altitude_;
    }
    return 0;
}