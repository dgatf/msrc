#include "pressure.h"

PressureInterface::PressureInterface(uint8_t pin, uint8_t alpha) : VoltageInterface(pin, alpha) {}

float PressureInterface::read(uint8_t index)
{
#ifdef SIM_SENSORS
    if (index == 0)
        return 25;
    return 0;
#endif
    if (index == 0)
    {
        if (millis() > 2000 && voltageOffset == 0)
        {
            for (int i = 0; i < 10; i++)
            {
                voltageOffset += readVoltage();
                delay(2);
            }
            voltageOffset = voltageOffset / 10;
        }
        float pressure = 1000 * (readVoltage() / (TRANSFER_SLOPE * TRANSFER_VCC) - voltageOffset);
        if (pressure < 0)
            pressure = 0;
        float airSpeed = sqrt(2 * pressure / AIR_DENS) / KNOT_TO_MS;
        value_ = calcAverage(alpha_ / 100.0F, value_, airSpeed);
        return value_;
    }
    return 0;
}