#include "sensor.h"

Sensor::Sensor(uint16_t dataId, float *valueLP, uint8_t refresh, AbstractDevice *deviceP) : dataId_(dataId), valueLP_(valueLP), refresh_(refresh), deviceP_(deviceP) {}

Sensor::~Sensor()
{
    delete deviceP_;
}

void Sensor::update()
{
    deviceP_->update();
}

uint32_t Sensor::valueFormatted()
{
    return formatData(dataId_, *valueLP_);
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

SensorDouble::SensorDouble(uint16_t dataId, float *valueLP, float *valueMP, uint8_t refresh, AbstractDevice *deviceP) : Sensor(dataId, valueLP, refresh, deviceP), valueMP_(valueMP) {}

uint32_t SensorDouble::valueFormatted()
{
    float valueLP = 0;
    float valueMP = 0;
    if (valueLP_ != NULL) valueLP = *valueLP_;
    if (valueMP_ != NULL) valueMP = *valueMP_;
    return formatData(dataId_, valueMP, valueLP);
}

SensorLatLon::SensorLatLon(uint16_t dataId, float *lonP, float *latP, uint8_t refresh, AbstractDevice *device) : SensorDouble(dataId, lonP, latP, refresh, device) {}

uint32_t SensorLatLon::valueFormatted()
{
    if (type_ == TYPE_LAT)
    {
        type_ = !type_;
        return formatLatLon(TYPE_LAT, *valueLP_);
    }
    else
    {
        type_ = !type_;
        return formatLatLon(TYPE_LON, *valueMP_);
    }
}

SensorDateTime::SensorDateTime(uint16_t dataId, float *timeP, float *dateP, uint8_t refresh, AbstractDevice *deviceP) : SensorDouble(dataId, timeP, dateP, refresh, deviceP) {}

uint32_t SensorDateTime::valueFormatted()
{
    if (!type_)
    {
        type_ = !type_;
        return formatDateTime(TYPE_DATE, *valueLP_);
    }
    else
    {
        type_ = !type_;
        return formatDateTime(TYPE_TIME, *valueMP_);
    }
}

SensorCell::SensorCell(uint16_t dataId, float *indexMP, float *indexLP, uint8_t cellIndex, uint8_t refresh, AbstractDevice *deviceP) : SensorDouble(dataId, indexMP, indexLP, refresh, deviceP), cellIndex_(cellIndex) {}

uint32_t SensorCell::valueFormatted()
{
    return formatCell(cellIndex_, *valueMP_, *valueLP_);
}