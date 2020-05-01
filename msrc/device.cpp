#include "device.h"

AbstractDevice::AbstractDevice() {}

float AbstractDevice::calcAverage(float alpha, float value, float newValue)
{
    return value + alpha * (newValue - value);
}