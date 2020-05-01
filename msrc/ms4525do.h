#ifndef MS4525DO_H
#define MS4525DO_H

#define MS4525DO_ADDR 0x28

#include <Arduino.h>
#include <Wire.h>
#include "device.h"
#include "i2c.h"

class MS4525DOInterface : public AbstractDevice, public I2C
{
private:
    uint8_t _address = MS4525DO_ADDR;
public:
    float temperature, pressure;
    MS4525DOInterface();
    bool begin();
    bool read();
};

#endif