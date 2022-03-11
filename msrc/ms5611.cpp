#include "ms5611.h"

MS5611::MS5611(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef) : device_(device), alphaTemp_(alphaTemp), alphaDef_(alphaDef) {}

void MS5611::begin()
{
    writeBytes(device_, MS5611_CMD_RESET);
    delay(5);
    C1_ = readUInt(device_, MS5611_CMD_READ_PROM, I2C_BIG_ENDIAN);
    C2_ = readUInt(device_, MS5611_CMD_READ_PROM + 1, I2C_BIG_ENDIAN);
    C3_ = readUInt(device_, MS5611_CMD_READ_PROM + 2, I2C_BIG_ENDIAN);
    C4_ = readUInt(device_, MS5611_CMD_READ_PROM + 3, I2C_BIG_ENDIAN);
    C5_ = readUInt(device_, MS5611_CMD_READ_PROM + 4, I2C_BIG_ENDIAN);
    C6_ = readUInt(device_, MS5611_CMD_READ_PROM + 5, I2C_BIG_ENDIAN);
}

void MS5611::calcPressure()
{
    int32_t dT, TEMP, T2;
    int64_t OFF, SENS, OFF2, SENS2;
    dT = D2_ - ((uint32_t)C5_ << 8);
    TEMP = 2000 + ((dT * C6_) >> 23);
    OFF = ((uint32_t)C2_ << 16) + ((C4_ * dT) >> 7);
    SENS = ((uint32_t)C1_ << 15) + ((C3_ * dT) >> 8);

    if (TEMP < 2000)
    {
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;
    }
    if (TEMP < 1500)
    {
        OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
        SENS2 = SENS2 + 11 * (TEMP + 1500) * (TEMP + 1500) / 2;
    }
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    int32_t P = (((D1_ * SENS) >> 21) - OFF) >> 15;
    temperature_ = (float)TEMP / 100; // °C
    pressure_ = (float)P / 100;

    if (P0_ == 0 && millis() > 5000)
#ifdef SIM_SENSORS
        P0_ = 1000;
#else
        P0_ = pressure_;
#endif
}

void MS5611::update()
{
    static uint16_t ts = 0;
    static bool isConvertingPressure = false;
    uint8_t data[3];
    if ((uint16_t)(millis() - ts) < MS5611_MEASUREMENT_INTERVAL)
        return;
    readBytes(device_, MS5611_CMD_ADC_READ, data, 3);
    if (isConvertingPressure)
        D1_ = (uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2]; // pressure
    else
        D2_ = (uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2]; // temperature
#ifdef SIM_SENSORS
    temperature_ = 20;
    pressure_ = 500;
    altitude_ = 1234.56;
    vario_ = 23.45;
#else
    isConvertingPressure = !isConvertingPressure;
    if (isConvertingPressure)
        writeBytes(device_, MS5611_CMD_CONV_D1); // pressure
    else
        writeBytes(device_, MS5611_CMD_CONV_D2); // temperature
    if (!isConvertingPressure)
    {
        calcPressure();
        altitude_ = calcAltitude(pressure_, temperature_, P0_);
        vario_ = calcSpeed(altitude_, MS5611_VARIO_INTERVAL);
    }
#endif
    ts = millis();
}

float *MS5611::temperatureP()
{
    return &temperature_;
}

float *MS5611::pressureP()
{
    return &pressure_;
}

float *MS5611::altitudeP()
{
    return &altitude_;
}

float *MS5611::varioP()
{
    return &vario_;
}