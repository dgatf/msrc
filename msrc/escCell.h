#ifndef ESCCELL_H
#define ESCCELL_H

#include <Arduino.h>

class EscCell
{
private:
protected:
    uint8_t cellCount_ = 0xFF;
    float cellVoltage_ = 0;
    uint8_t setCellCount(float voltage);

public:
    EscCell();
    float *cellVoltageP();
};

#endif