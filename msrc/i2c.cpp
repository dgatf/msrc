#include "i2c.h"

I2C::I2C() {}

int16_t I2C::readInt(uint8_t device, uint8_t reg, uint8_t endian)
{
    uint8_t data[2] = {0};
    readBytes(device, reg, data, 2);
    if (endian == BIG_ENDIAN)
        return data[0] << 8 | data[1];
    return data[1] << 8 | data[0];
}

uint16_t I2C::readUInt(uint8_t device, uint8_t reg, uint8_t endian)
{
    uint8_t data[2] = {0};
    readBytes(device, reg, data, 2);
    if (endian == BIG_ENDIAN)
        return data[0] << 8 | data[1];
    return data[1] << 8 | data[0];
}

uint8_t I2C::readBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t lenght)
{
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.endTransmission(true);
    Wire.requestFrom(device, lenght);
    return Wire.readBytes(data, lenght);
}

uint8_t I2C::writeBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t lenght)
{
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.write(data, lenght);
    return Wire.endTransmission();
}

/*uint8_t I2C::scan(uint8_t *devices)
{
    uint8_t error, address;
    uint8_t cont = 0;
    _wire.setTimeout(1);
    for (address = 1; address < 127; address++)
    {
        _wire.beginTransmission(address);
        error = _wire.endTransmission(true);
        if (error == 0)
        {
            devices[cont] = address;
            cont++;   
        }
    }
    return cont;
}*/