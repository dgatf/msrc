#ifndef FUNCTIONS_H
#define FUNCITONS_H

#include <Arduino.h>

#include "constants.h"
#include "softserial.h"
#include "hardserial.h"

uint8_t setCellCount(float voltage);
float calcAverage(float alpha, float oldValue, float newValue);

class Consumption
{
private:
public:
    float calcConsumption(float current, uint16_t currentMax = 0);
};

class Vario
{
private:
public:
    float calcSpeed(float altitude, uint16_t interval = 0);
};

#endif