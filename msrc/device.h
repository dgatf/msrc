#ifndef DEVICE_H
#define DEVICE_H

#include <Arduino.h>
#include "config.h"
#include "functions.h"

class AbstractDevice
{
private:
protected:
public:
    AbstractDevice();
    virtual ~AbstractDevice();
    virtual void update() = 0;
};

#endif