#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "device.h"
#include "formatData.h"

class Sensor : public FormatData
{
protected:
    uint16_t timestamp_ = 0, dataId_, frameId_ = 0x10;
    float valueL_ = 0;
    uint32_t value_;
    uint8_t indexL_ = 0;
    uint8_t refresh_;
    AbstractDevice *device_;

public:
    Sensor(uint16_t dataId, uint8_t indexL, uint8_t refresh, AbstractDevice *device);
    Sensor(uint16_t dataId, uint8_t refresh, AbstractDevice *device);
    virtual ~Sensor();
    Sensor *nextP = NULL;
    uint16_t timestamp();
    void setTimestamp(uint16_t dataId);
    uint16_t dataId();
    uint16_t frameId();
    uint8_t refresh();
    virtual void update();
    virtual uint32_t value();
    float valueL();
};

class SensorDouble : public Sensor
{
protected:
    float valueM_ = 0;
    uint8_t indexM_ = 255;

public:
    SensorDouble(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t refresh, AbstractDevice *device);
    float valueM();
    virtual void update();
    virtual uint32_t value();
};

class SensorLatLon : public SensorDouble
{
protected:
    uint8_t type_ = 0;

public:
    SensorLatLon(uint16_t dataId, uint8_t indexLon, uint8_t indexLat, uint8_t refresh, AbstractDevice *device);
    uint32_t value();
};

class SensorDateTime : public SensorDouble
{
protected:
    uint8_t type_ = 0;

public:
    SensorDateTime(uint16_t dataId, uint8_t indexTime, uint8_t indexDate, uint8_t refresh, AbstractDevice *device);
    uint32_t value();
    void update();
};

class SensorCell : public SensorDouble
{
protected:
    uint8_t cellIndex_ = 0;

public:
    SensorCell(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t cellIndex, uint8_t refresh, AbstractDevice *device);
    void update();
    uint32_t value();
};

#endif