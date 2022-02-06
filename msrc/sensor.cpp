#include "sensor.h"

Sensor::Sensor(uint16_t dataId, float *valueLP, uint8_t refresh, AbstractDevice *deviceP) : dataId_(dataId), valueLP_(valueLP), refresh_(refresh), deviceP_(deviceP) {}

Sensor::~Sensor()
{
}

void Sensor::update()
{
    deviceP_->update();
}

uint32_t Sensor::valueFormatted()
{
#ifdef ESC_SIGNATURE
    if (dataId_ > DIY_STREAM_FIRST_ID && dataId_  < DIY_LAST_ID)
    {
        uint32_t buffer;
        memcpy(&buffer, valueLP_, 4);
        return buffer;
    }
#endif
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
    return formatData(dataId_, valueLP, valueMP);
}

SensorLatLon::SensorLatLon(uint16_t dataId, float *lonP, float *latP, uint8_t refresh, AbstractDevice *device) : SensorDouble(dataId, lonP, latP, refresh, device) {}

uint32_t SensorLatLon::valueFormatted()
{
    if (type_ == TYPE_LAT)
    {
        type_ = !type_;
        return formatLatLon(TYPE_LAT, *valueMP_);
    }
    else
    {
        type_ = !type_;
        return formatLatLon(TYPE_LON, *valueLP_);
    }
}

SensorDateTime::SensorDateTime(uint16_t dataId, float *timeP, float *dateP, uint8_t refresh, AbstractDevice *deviceP) : SensorDouble(dataId, timeP, dateP, refresh, deviceP) {}

uint32_t SensorDateTime::valueFormatted()
{
    if (!type_)
    {
        type_ = !type_;
        return formatDateTime(TYPE_DATE, *valueMP_);
    }
    else
    {
        type_ = !type_;
        return formatDateTime(TYPE_TIME, *valueLP_);
    }
}

SensorCell::SensorCell(uint16_t dataId, float *indexMP, float *indexLP, uint8_t cellIndex, uint8_t refresh, AbstractDevice *deviceP) : SensorDouble(dataId, indexMP, indexLP, refresh, deviceP), cellIndex_(cellIndex) {}

uint32_t SensorCell::valueFormatted()
{
    return formatCell(cellIndex_, *valueMP_, *valueLP_);
}


Sensord::Sensord(uint8_t dataId, float *valueP, uint8_t refresh, AbstractDevice *deviceP) : dataId_(dataId), valueP_(valueP), refresh_(refresh), deviceP_(deviceP) {}

Sensord::~Sensord()
{
}

void Sensord::update()
{
    deviceP_->update();
}

uint16_t Sensord::valueFormatted()
{
    return formatData(dataId_, *valueP_);
}

uint16_t Sensord::timestamp()
{
    return timestamp_;
}

void Sensord::setTimestamp(uint16_t timestamp)
{
    timestamp_ = timestamp;
}

uint8_t Sensord::dataId()
{
    return dataId_;
}

uint8_t Sensord::refresh()
{
    return refresh_;
}

SensorIbus::SensorIbus(uint8_t dataId, uint8_t type, AbstractDevice *deviceP) : dataId_(dataId), type_(type), deviceP_(deviceP) {}

SensorIbus::~SensorIbus()
{
}

void SensorIbus::update()
{
    deviceP_->update();
}

uint8_t SensorIbus::dataId()
{
    return dataId_;
}

uint8_t SensorIbus::type()
{
    return type_;
}

SensorIbusS16::SensorIbusS16(uint8_t dataId, uint8_t type, float *valueP, AbstractDevice *deviceP) : SensorIbus(dataId, type, deviceP), valueP_(valueP) {}

SensorIbusS16::~SensorIbusS16()
{
}

uint8_t *SensorIbusS16::valueFormatted()
{
    valueFormatted_ = formatIbus(dataId_, *valueP_);
    return (uint8_t *)&valueFormatted_;
}

SensorIbusU16::SensorIbusU16(uint8_t dataId, uint8_t type, float *valueP, AbstractDevice *deviceP) : SensorIbus(dataId, type, deviceP), valueP_(valueP) {}

SensorIbusU16::~SensorIbusU16()
{
}

uint8_t *SensorIbusU16::valueFormatted()
{
    valueFormatted_ = formatIbus(dataId_, *valueP_);
    return (uint8_t *)&valueFormatted_;
}

SensorIbusS32::SensorIbusS32(uint8_t dataId, uint8_t type, float *valueP, AbstractDevice *deviceP) : SensorIbus(dataId, type, deviceP), valueP_(valueP) {}

SensorIbusS32::~SensorIbusS32()
{
}

uint8_t *SensorIbusS32::valueFormatted()
{
    valueFormatted_ = formatIbus(dataId_, *valueP_);
    return (uint8_t *)&valueFormatted_;
}

SensorIbusGps::SensorIbusGps(uint8_t dataId, uint8_t type, float *valueLatP, float *valueLonP, float *valueAltP, AbstractDevice *deviceP) : SensorIbus(dataId, type, deviceP), valueLatP_(valueLatP), valueLonP_(valueLonP), valueAltP_(valueAltP) {}

SensorIbusGps::~SensorIbusGps()
{
}

uint8_t *SensorIbusGps::valueFormatted()
{
    int32_t value;
    value = formatIbus(AFHDS2A_ID_GPS_LAT, *valueLatP_);
    memcpy(buffer + 3, &value, 4);
    value = formatIbus(AFHDS2A_ID_GPS_LON, *valueLonP_);
    memcpy(buffer + 7, &value, 4);
    value = formatIbus(AFHDS2A_ID_GPS_ALT, *valueAltP_);
    memcpy(buffer + 11, &value, 4);
    return (uint8_t *)&buffer;
}

SensorSbus::SensorSbus(uint8_t dataId, float *valueP, AbstractDevice *deviceP) : dataId_(dataId), valueP_(valueP), deviceP_(deviceP) {}

SensorSbus::~SensorSbus()
{
}

void SensorSbus::update()
{
    deviceP_->update();
}

uint16_t SensorSbus::valueFormatted()
{
    if (valueP_)
        return formatSbus(dataId_, *valueP_);
    return formatSbus(dataId_, 0);
}

uint8_t SensorSbus::dataId()
{
    return dataId_;
}

float *SensorSbus::valueP()
{
    return valueP_;
}

SensorMultiplex::SensorMultiplex(uint8_t dataId, float *valueP, AbstractDevice *deviceP) : dataId_(dataId), valueP_(valueP), deviceP_(deviceP) {}

SensorMultiplex::~SensorMultiplex()
{
}

void SensorMultiplex::update()
{
    deviceP_->update();
}

uint16_t SensorMultiplex::valueFormatted()
{
    return formatMultiplex(dataId_, *valueP_);
}

uint8_t SensorMultiplex::dataId()
{
    return dataId_;
}

float *SensorMultiplex::valueP()
{
    return valueP_;
}

SensorJetiEx::SensorJetiEx(uint8_t type, uint8_t format, float *valueP, AbstractDevice *deviceP) : type_(type), format_(format), valueP_(valueP), deviceP_(deviceP) {}

SensorJetiEx::~SensorJetiEx()
{
}

void SensorJetiEx::update()
{
    deviceP_->update();
}

float *SensorJetiEx::valueP()
{
    return valueP_;
}

void SensorJetiEx::setText(const char *textP)
{
    strncpy(text_, textP, 32);
}

void SensorJetiEx::setUnit(const char *textP)
{
    strncpy(unit_, textP, 8);
}

char *SensorJetiEx::textP()
{
    return text_;
}

char *SensorJetiEx::unitP()
{
    return unit_;
}

void SensorJetiEx::setSensorId(uint8_t sensorId)
{
    sensorId_ = sensorId;
}

uint8_t SensorJetiEx::type()
{
    return type_;
}

uint8_t SensorJetiEx::format()
{
    return format_;
}
