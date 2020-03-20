#ifndef ESCPWM_H
#define ESCPWM_H

#define COMP_TO_MICROS ((float)8000000UL / F_CPU)
#define PWM_IN_TRIGGER_MICROS 30000
#define PWM_IN_TRIGGER_PULSES 10

#include <Arduino.h>
#include "smartport.h"

class EscPWMInterface : public AbstractDevice
{
private:
    uint8_t alphaRpm_;
    float rpm_;

protected:
public:
    EscPWMInterface(uint8_t alphaRpm);
    void begin();
    float read(uint8_t index);
};

#endif