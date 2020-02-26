#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <Wire.h>
#include "bmp.h"

class MS5611Interface : public Bmp
{
private:
public:
    MS5611Interface(uint8_t address, uint8_t alphaTemp, uint8_t alphaDef);
    bool begin();
    bool readTemperature();
    bool readPressure();
};

#endif