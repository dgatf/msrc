#include "sensor.h"

Sensor::Sensor(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), indexL_(indexL), indexM_(indexM), refresh_(refresh), device_(device) {}
Sensor::Sensor(uint16_t dataId, uint8_t indexL, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), indexL_(indexL), refresh_(refresh), device_(device) {}
Sensor::Sensor(uint16_t dataId, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), refresh_(refresh), device_(device) {}

Sensor::~Sensor()
{
    delete device_;
}

float Sensor::read(uint8_t index)
{
    return device_->read(index);
}

uint8_t Sensor::indexL()
{
    return indexL_;
}

uint8_t Sensor::indexM()
{
    return indexM_;
}

uint16_t Sensor::timestamp()
{
    return timestamp_;
}

void Sensor::setTimestamp(uint16_t timestamp)
{
    timestamp_ = timestamp;
}

uint16_t Sensor::dataId()
{
    return dataId_;
}

uint16_t Sensor::frameId()
{
    return frameId_;
}

uint8_t Sensor::refresh()
{
    return refresh_;
}

float Sensor::valueM()
{
    return valueM_;
}

void Sensor::setValueM(float value)
{
    valueM_ = value;
}

float Sensor::valueL()
{
    return valueL_;
}

void Sensor::setValueL(float value)
{
    valueL_ = value;
}