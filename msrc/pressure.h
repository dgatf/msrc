#ifndef PRESSURE_H
#define PRESSURE_H

#define AIR_DENS 1.204  // 20ºC, 1atm
#define KNOT_TO_MS 1.94384
// Transfer function: P(Pa) = 1000 * (Vo/(SLOPE*VCC) - voltageOffset)
// MPXV7002
#define TRANSFER_SLOPE 0.2
#define TRANSFER_VCC 5

#include <Arduino.h>
#include "voltage.h"

class PressureInterface : public VoltageInterface
{
private:
    float voltageOffset = 0;
public:
    PressureInterface(uint8_t pin, uint8_t alpha);
    float read(uint8_t index);
};

#endif