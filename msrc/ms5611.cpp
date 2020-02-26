#include "ms5611.h"

MS5611Interface::MS5611Interface(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef) : Bmp(device, alphaTemp, alphaDef)  {}

bool MS5611Interface::begin()
{
    return 0;
}

bool MS5611Interface::readTemperature()
{
    return 0;
}

bool MS5611Interface::readPressure()
{
    return 0;
}
