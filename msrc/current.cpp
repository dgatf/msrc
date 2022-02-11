#include "current.h"

Current::Current(uint8_t pin, uint8_t alpha, float multiplier) : Voltage(pin, alpha, multiplier) {}

Current::Current(uint8_t pin, uint8_t alpha) : Voltage(pin, alpha) {}

void Current::update()
{
    value_ = calcAverage(alpha_ / 100.0F, value_, readVoltage());
    consumption_ = calcConsumption(value_);
#ifdef SIM_SENSORS
    consumption_ = 12.34;
#endif
}

float *Current::consumptionP()
{
    return &consumption_;
}