#ifndef VOLTAGE_H
#define VOLTAGE_H

#if F_CPU == 16000000UL
#define BOARD_VCC 5
#else
#define BOARD_VCC 3.3
#endif

#define ADC_RESOLUTION 1024.0

#include <Arduino.h>
#include "device.h"

class Voltage : public AbstractDevice
{
protected:
    uint8_t pin_;
    float value_ = 0;
    uint8_t alpha_;
    float multiplier_ = 1;
    float readVoltage();

public:
    Voltage(uint8_t pin, uint8_t alpha, float multiplier);
    Voltage(uint8_t pin, uint8_t alpha);
    virtual void update();
    float *valueP();
};

#endif