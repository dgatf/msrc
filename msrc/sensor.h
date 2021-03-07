#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "device.h"
#include "formatData.h"

class Sensor : public FormatData
{
protected:
    uint16_t timestamp_ = 0, dataId_, frameId_ = 0x10;
    float *valueLP_;
    uint32_t value_;
    uint8_t refresh_;

public:
    AbstractDevice *deviceP_;
    Sensor(uint16_t dataId, float *valueLP, uint8_t refresh, AbstractDevice *deviceP);
    virtual ~Sensor();
    Sensor *nextP = NULL;
    uint16_t timestamp();
    void setTimestamp(uint16_t dataId);
    uint16_t dataId();
    uint16_t frameId();
    uint8_t refresh();
    void update();
    virtual uint32_t valueFormatted();
};

class SensorDouble : public Sensor
{
protected:
    float *valueMP_;

public:
    SensorDouble(uint16_t dataId, float *valueLP, float *valueMP, uint8_t refresh, AbstractDevice *deviceP);
    virtual uint32_t valueFormatted();
};

class SensorLatLon : public SensorDouble
{
protected:
    uint8_t type_ = TYPE_LAT;

public:
    SensorLatLon(uint16_t dataId, float *lonP, float *latP, uint8_t refresh, AbstractDevice *deviceP);
    uint32_t valueFormatted();
};

class SensorDateTime : public SensorDouble
{
protected:
    uint8_t type_ = TYPE_DATE;

public:
    SensorDateTime(uint16_t dataId, float *timeP, float *dateP, uint8_t refresh, AbstractDevice *deviceP);
    uint32_t valueFormatted();
};

class SensorCell : public SensorDouble
{
protected:
    uint8_t cellIndex_ = 0;

public:
    SensorCell(uint16_t dataId, float *indexM, float *indexL, uint8_t cellIndex, uint8_t refresh, AbstractDevice *deviceP);
    uint32_t valueFormatted();
};

#endif