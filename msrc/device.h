#ifndef DEVICE_H
#define DEVICE_H

#include <Arduino.h>
#include "constants.h"
#include "functions.h"
#include "softserial.h"
#include "hardserial.h"

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