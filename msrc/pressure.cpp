#include "pressure.h"

Pressure::Pressure(uint8_t pin, uint8_t alpha) : Voltage(pin, alpha) {}

void Pressure::update()
{
    if (RX_PROTOCOL != RX_SBUS) // Auto offset is done in Futaba transmitters
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
    }
    float pressure = 1000 * (readVoltage() / (TRANSFER_SLOPE * TRANSFER_VCC) - voltageOffset);
    if (pressure < 0)
        pressure = 0;
#ifdef SIM_SENSORS
    value_ = 123.45;
#else
    float airSpeed = sqrt(2 * pressure / AIR_DENS) / KNOT_TO_MS;
    value_ = calcAverage(alpha_ / 100.0F, value_, airSpeed);
#endif
}