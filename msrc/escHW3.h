#ifndef ESCHW3_H
#define ESCHW3_H

#include <Arduino.h>
#include "smartport.h"

class EscHW3Interface : public AbstractDevice
{
private:
    uint8_t thr_ = 0, pwm_ = 0, alphaRpm_;
    float rpm_;
    Stream &serial_;
protected:  
public:
    EscHW3Interface(Stream &serial, uint8_t alphaRpm);
    bool update();
    float read(uint8_t index);
};

#endif