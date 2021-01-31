#include "escCell.h"

EscCell::EscCell() {}

uint8_t EscCell::setCellCount(float voltage)
{
    float level[] = {4.2, 8.4, 12.6, 16.8, 21, 25.2, 29.4, 33.6, 33.6, 42, 42};
    int8_t cont = 10;
    while (voltage < level[cont])
        cont--;
    return cont + 2;
}

float *EscCell::cellVoltageP()
{
    return &cellVoltage_;
}