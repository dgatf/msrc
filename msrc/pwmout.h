#ifndef PWMOUT_H
#define PWMOUT_H

#include <Arduino.h>
#include "config.h"

class PwmOut
{
private:
    static bool isEnabled_;
    static float *rpmP_;
    void stop();
public:
    PwmOut();
    void enable();
    void disable();
    void update();
    void setRpmP(float *rpmP);
};

#endif