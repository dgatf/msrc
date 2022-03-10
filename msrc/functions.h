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
    uint16_t prevMs = 0;
public:
    float calcConsumption(float current, uint16_t currentMax = 0);
};

class Vario
{
private:
    uint16_t prevMs = 0;
    float prevAltitude = 0;
    float speed = 0;
public:
    float calcSpeed(float altitude, uint16_t intervalMin = 0);
    float calcAltitude(float pressure, float temperature, float P0);
};

#endif