#include "voltage.h"

Voltage::Voltage(uint8_t pin, uint8_t alpha, float multiplier) : pin_(pin), alpha_(alpha), multiplier_(multiplier) {}

Voltage::Voltage(uint8_t pin, uint8_t alpha) : pin_(pin), alpha_(alpha) {}

float Voltage::readVoltage()
{
    return analogRead(pin_) * BOARD_VCC / ADC_RESOLUTION * multiplier_;
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