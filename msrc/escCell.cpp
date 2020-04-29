#include "escCell.h"

EscCell::EscCell() {}

uint8_t EscCell::setCellCount(float voltage)
{
    if (voltage > 42)
        return 12;
    if (voltage > 33.6)
        return 10;
    if (voltage > 29.4)
        return 8;
    if (voltage > 25.2)
        return 7;
    if (voltage > 21)
        return 6;
    if (voltage > 16.8)
        return 5;
    if (voltage > 12.6)
        return 4;
    if (voltage > 8.4)
        return 3;
    if (voltage > 4.2)
        return 2;
    return 1;
}
