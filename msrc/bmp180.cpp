#include "bmp180.h"

Bmp180Interface::Bmp180Interface(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef) : Bmp(device, alphaTemp, alphaDef) {}

bool Bmp180Interface::begin()
{
    float c3, c4, b1;
    AC1_ = readInt(address_, 0xAA, BIG_ENDIAN);
    AC2_ = readInt(address_, 0xAC, BIG_ENDIAN);
    AC3_ = readInt(address_, 0xAE, BIG_ENDIAN);
    AC4_ = readUInt(address_, 0xB0, BIG_ENDIAN);
    AC5_ = readUInt(address_, 0xB2, BIG_ENDIAN);
    AC6_ = readUInt(address_, 0xB4, BIG_ENDIAN);
    VB1_ = readInt(address_, 0xB6, BIG_ENDIAN);
    VB2_ = readInt(address_, 0xB8, BIG_ENDIAN);
    MB_ = readInt(address_, 0xBA, BIG_ENDIAN);
    MC_ = readInt(address_, 0xBC, BIG_ENDIAN);
    MD_ = readInt(address_, 0xBE, BIG_ENDIAN);
    c3 = 160.0 * pow(2, -15) * AC3_;
    c4 = pow(10, -3) * pow(2, -15) * AC4_;
    b1 = pow(160, 2) * pow(2, -30) * VB1_;
    c5_ = (pow(2, -15) / 160) * AC5_;
    c6_ = AC6_;
    mc_ = (pow(2, 11) / pow(160, 2)) * MC_;
    md_ = MD_ / 160.0;
    x0_ = AC1_;
    x1_ = 160.0 * pow(2, -13) * AC2_;
    x2_ = pow(160, 2) * pow(2, -25) * VB2_;
    y0_ = c4 * pow(2, 15);
    y1_ = c4 * c3;
    y2_ = c4 * b1;
    p0_ = (3791.0 - 8.0) / 1600.0;
    p1_ = 1.0 - 7357.0 * pow(2, -20);
    p2_ = 3038.0 * 100.0 * pow(2, -36);
}

uint8_t Bmp180Interface::startTemperature()
{
    uint8_t data[1] = {BMP180_COMMAND_TEMPERATURE};
    writeBytes(address_, BMP_REG_CONTROL, data, 2);
    return true;
}

uint8_t Bmp180Interface::startPressure()
{
    uint8_t data[2], delay;

    data[0] = BMP_REG_CONTROL;

    switch (sampling_)
    {
    case 0:
        data[1] = BMP180_COMMAND_PRESSURE0;
        delay = 5;
        break;
    case 1:
        data[1] = BMP180_COMMAND_PRESSURE1;
        delay = 8;
        break;
    case 2:
        data[1] = BMP180_COMMAND_PRESSURE2;
        delay = 14;
        break;
    case 3:
        data[1] = BMP180_COMMAND_PRESSURE3;
        delay = 26;
        break;
    default:
        data[1] = BMP180_COMMAND_PRESSURE0;
        delay = 5;
        break;
    }
    writeBytes(address_, BMP180_COMMAND_PRESSURE0, data, 2);
    return 1;
}

bool Bmp180Interface::readTemperature()
{
    if (state_ == BMP180_STATE_READY)
    {
        state_ = BMP180_STATE_WAIT_TEMP;
        delay_ = startTemperature();
        timestamp_ = millis();
        return false;
    }
    if (state_ == BMP180_STATE_WAIT_PRESS)
    {
        return false;
    }
    if (state_ == BMP180_STATE_WAIT_TEMP && millis() - timestamp_ > delay_)
    {
        uint8_t data[2];
        char result;
        float tu, a, t;
        result = readBytes(address_, BMP180_REG_RESULT, data, 2);
        if (result) // good read, calculate temperature
        {
            tu = (data[0] * 256.0) + data[1];
            a = c5_ * (tu - c6_);
            t = a + (mc_ / (a + md_));
            temperature_ = t; //calcAverage(alphaTemp_, temperature_, t);
        }
        return result;
    }
    return 5;
}

bool Bmp180Interface::readPressure()
{
    if (state_ == BMP180_STATE_READY)
    {
        state_ = BMP180_STATE_WAIT_PRESS;
        delay_ = startPressure();
        timestamp_ = millis();
        return false;
    }
    if (state_ == BMP180_STATE_WAIT_TEMP)
    {
        return false;
    }
    if (state_ == BMP180_STATE_WAIT_PRESS && millis() - timestamp_ > delay_)
    {
        uint8_t data[3];
        uint8_t result;
        float pu, s, x, y, z, p;
        result = readBytes(address_, BMP180_REG_RESULT, data, 3);
        if (result)
        {
            pu = (data[0] * 256.0) + data[1] + (data[2] / 256.0);
            s = temperature_ - 25.0;
            x = (x2_ * pow(s, 2)) + (x1_ * s) + x0_;
            y = (y2_ * pow(s, 2)) + (y1_ * s) + y0_;
            z = (pu - x) / y;
            p = (p2_ * pow(z, 2)) + (p1_ * z) + p0_;
            pressure_ = p; //calcAverage(alphaDef_, pressure_, p);
        }
        return result;
    }
    return false;
}

float Bmp180Interface::read(uint8_t index)
{
    if (index == BMP_TEMPERATURE)
    {
#ifdef SIM_SENSORS
        return 25;
#endif
        readTemperature();
        return temperature_;
    }
    if (index == BMP_ALTITUDE)
    {
#ifdef SIM_SENSORS
        return 126;
#endif
        if (readPressure())
        {
            calcAltitude();
        }
        return altitude_;
    }
    return 0;
}
