#ifndef ESCCELL_H
#define ESCCELL_H

#include <Arduino.h>
#include "smartport.h"

class EscCell
{
private:
protected:
    uint8_t cellCount_ = 0xFF;
    uint8_t setCellCount(float voltage);

public:
    EscCell();
};

#endif