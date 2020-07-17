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
        float pressure = 1000 * BOARD_VCC * (0.2 * readVoltage() - 0.5); // MPXV7002
        if (pressure < 0) pressure = 0;
        float airSpeed = sqrt(2 * pressure / AIR_DENS) / KNOT_TO_MS;
        value_ = calcAverage(alpha_ / 100.0F, value_, airSpeed);
        return value_;
    }
    return 0;
}