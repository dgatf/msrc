#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "device.h"

class Sensor
{
protected:
    uint16_t timestamp_ = 0, dataId_, frameId_ = 0x10;
    float valueL_ = 0, valueM_ = 0;
    uint8_t indexL_ = 0, indexM_ = 255;
    uint8_t refresh_;
    AbstractDevice *device_;

public:
    Sensor(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t refresh, AbstractDevice *device);
    Sensor(uint16_t dataId, uint8_t indexL, uint8_t refresh, AbstractDevice *device);
    Sensor(uint16_t dataId, uint8_t refresh, AbstractDevice *device);
    virtual ~Sensor();
    Sensor *nextP = NULL;
    uint16_t timestamp();
    void setTimestamp(uint16_t dataId);
    uint16_t dataId();
    uint16_t frameId();
    uint8_t refresh();
    uint8_t indexL();
    uint8_t indexM();
    float valueL();
    void setValueL(float valueM);
    float valueM();
    void setValueM(float valueM);
    float read(uint8_t index);
};

#endif