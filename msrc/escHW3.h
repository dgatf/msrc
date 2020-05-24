#ifndef ESCHW3_H
#define ESCHW3_H

#define ESCSERIAL_TIMEOUT 3

//#define DEBUG_ESC

#include <Arduino.h>
#include "device.h"

class EscHW3Interface : public AbstractDevice
{
private:
    uint8_t thr_ = 0, pwm_ = 0, alphaRpm_;
    float rpm_;
    HardwareSerial &serial_;

protected:
public:
    EscHW3Interface(HardwareSerial &serial, uint8_t alphaRpm);
    void begin();
    bool update();
    float read(uint8_t index);
};

#endif