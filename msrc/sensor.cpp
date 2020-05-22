#include "sensor.h"

Sensor::Sensor(uint16_t dataId, uint8_t indexL, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), indexL_(indexL), refresh_(refresh), device_(device) {}
Sensor::Sensor(uint16_t dataId, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), refresh_(refresh), device_(device) {}

Sensor::~Sensor()
{
    delete device_;
}

void Sensor::update()
{
    valueL_ = device_->read(indexL_);
}

uint32_t Sensor::value()
{
    value_ = formatData(dataId_, valueL_);
    return value_;
}

float Sensor::valueL()
{
    return valueL_;
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

SensorDouble::SensorDouble(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t refresh, AbstractDevice *device) : Sensor(dataId, indexL, refresh, device), indexM_(indexM) {}

void SensorDouble::update()
{
    valueL_ = device_->read(indexL_);
    valueM_ = device_->read(indexM_);
}

uint32_t SensorDouble::value()
{
    value_ = formatData(dataId_, valueM_, valueL_);
    return value_;
}

float SensorDouble::valueM()
{
    return valueM_;
}

SensorLatLon::SensorLatLon(uint16_t dataId, uint8_t indexLon, uint8_t indexLat, uint8_t refresh, AbstractDevice *device) : SensorDouble(dataId, indexLon, indexLat, refresh, device) {}

uint32_t SensorLatLon::value()
{
    if (type_)
    {
        value_ = formatLatLon(TYPE_LAT, valueL_);
    }
    else
    {
        value_ = formatLatLon(TYPE_LON, valueM_);
    }
    type_ = !type_;
    return value_;
}

SensorDateTime::SensorDateTime(uint16_t dataId, uint8_t indexTime, uint8_t indexDate, uint8_t refresh, AbstractDevice *device) : SensorDouble(dataId, indexTime, indexDate, refresh, device) {}

void SensorDateTime::update()
{
    valueL_ = device_->read(indexL_);
    valueM_ = device_->read(indexM_);
}

uint32_t SensorDateTime::value()
{
    if (type_)
    {
        value_ = formatDateTime(TYPE_DATE, valueL_);
    }
    else
    {
        value_ = formatDateTime(TYPE_TIME, valueM_);
    }
    type_ = !type_;
    return value_;
}

SensorCell::SensorCell(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t cellIndex, uint8_t refresh, AbstractDevice *device) : SensorDouble(dataId, indexM, indexL, refresh, device), cellIndex_(cellIndex) {}

void SensorCell::update()
{
    valueL_ = device_->read(indexL_);
    valueM_ = device_->read(indexM_);
}

uint32_t SensorCell::value()
{
    value_ = formatCell(cellIndex_, valueM_, valueL_);
    return value_;
}