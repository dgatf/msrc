#ifndef ESCHW3_H
#define ESCHW3_H

#define ESCHWV3_ESCSERIAL_TIMEOUT 2000
#define ESCHWV3_PACKET_LENGHT 10

#include <Arduino.h>
#include "device.h"

class EscHW3 : public AbstractDevice
{
private:
    uint8_t alphaRpm_;
    float rpm_;
    AbstractSerial &serial_;

protected:
public:
    EscHW3(AbstractSerial &serial, uint8_t alphaRpm);
    void begin();
    void update();
    float *rpmP();
};

#endif