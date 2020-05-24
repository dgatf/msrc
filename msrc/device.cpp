#include "device.h"

AbstractDevice::AbstractDevice() {}

AbstractDevice::~AbstractDevice() {}

float AbstractDevice::calcAverage(float alpha, float oldValue, float newValue)
{
    return (1 - alpha) * oldValue + alpha * newValue;
}