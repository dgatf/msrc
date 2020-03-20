#ifndef BMP_H
#define BMP_H

#define BMP_REG_CONTROL 0xF4

#define BMP_TEMPERATURE 0
#define BMP_ALTITUDE 1

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "smartport.h"
#include "i2c.h"

class Bmp : public AbstractDevice, public I2C
{
protected:
        float P0_ = 500; //101325;
        float pressure_ = 0, altitude_ = 0, temperature_ = 0;
        uint8_t device_, alphaTemp_, alphaDef_;

public:
        Bmp(uint8_t device, uint8_t alphaTemp, uint8_t alphaDef);
        float calcAltitude();
};

#endif