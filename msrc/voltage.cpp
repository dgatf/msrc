#include "voltage.h"

Voltage::Voltage(uint8_t pin, uint8_t alpha) : pin_(pin), alpha_(alpha) {}

float Voltage::readVoltage()
{
    const float analogToVolt = (float)BOARD_VCC / 1024;
    uint16_t value = analogRead(pin_);
    return value * analogToVolt;
}

float Voltage::read(uint8_t index)
{
#ifdef SIM_SENSORS
    if (index == 0)
        return 34;
    return 0;
#endif
    if (index == 0)
    {
        value_ = calcAverage(alpha_ / 100.0F, value_, readVoltage());
        return value_;
    }
    return 0;
}