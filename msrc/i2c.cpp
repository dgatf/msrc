#include "i2c.h"

I2C::I2C() {}

int16_t I2C::readInt(uint8_t device, uint8_t reg, uint8_t endian)
{
    uint8_t data[2] = {0};
    readBytes(device, reg, data, 2);
    if (endian == I2C_BIG_ENDIAN)
        return data[0] << 8 | data[1];
    return data[1] << 8 | data[0];
}

uint16_t I2C::readUInt(uint8_t device, uint8_t reg, uint8_t endian)
{
    uint8_t data[2] = {0};
    readBytes(device, reg, data, 2);
    if (endian == I2C_BIG_ENDIAN)
        return data[0] << 8 | data[1];
    return data[1] << 8 | data[0];
}

uint8_t I2C::readBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t lenght)
{
    I2C_CHANNEL.beginTransmission(device);
    I2C_CHANNEL.write(reg);
    I2C_CHANNEL.endTransmission(true);
    I2C_CHANNEL.requestFrom(device, lenght);
    return I2C_CHANNEL.readBytes(data, lenght);
}

uint8_t I2C::writeBytes(uint8_t device, uint8_t reg)
{
    I2C_CHANNEL.beginTransmission(device);
    I2C_CHANNEL.write(reg);
    return I2C_CHANNEL.endTransmission();
}

uint8_t I2C::writeBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t lenght)
{
    I2C_CHANNEL.beginTransmission(device);
    I2C_CHANNEL.write(reg);
    I2C_CHANNEL.write(data, lenght);
    return I2C_CHANNEL.endTransmission();
}