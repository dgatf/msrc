#include "voltage.h"

Voltage::Voltage(uint8_t pin, uint8_t alpha) : pin_(pin), alpha_(alpha) {}

float Voltage::readVoltage()
{
    return analogRead(pin_) * BOARD_VCC / 1024.0;
}

void Voltage::update()
{
    value_ = calcAverage(alpha_ / 100.0F, value_, readVoltage());
#ifdef SIM_SENSORS
    value_ = 12.34;
#endif
}

float *Voltage::valueP()
{
    return &value_;
}