#include "voltage.h"

VoltageInterface::VoltageInterface(uint8_t pin, uint8_t alpha) : pin_(pin), alpha_(alpha) {}

float VoltageInterface::readVoltage()
{
    const float analogToVolt = (float)BOARD_VCC / 1024;
    uint16_t value = analogRead(pin_);
    return value * analogToVolt;
}

float VoltageInterface::read(uint8_t pin)
{
#ifdef SIM_SENSORS
    return 34;
#endif
    value_ = calcAverage(alpha_ / 100, value_, readVoltage());
    return value_;
}