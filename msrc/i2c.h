#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>

#define BIG_ENDIAN 1
#define LITTLE_ENDIAN 0

class I2C
{
private:
protected:
public:
    I2C();
    //uint8_t scan(uint8_t *devices);
    int16_t readInt(uint8_t device, uint8_t reg, uint8_t endian);
    uint16_t readUInt(uint8_t device, uint8_t reg, uint8_t endian);
    uint8_t readBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t length);
    uint8_t writeBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t length);
    uint8_t writeBytes(uint8_t device, uint8_t reg);
};

#endif