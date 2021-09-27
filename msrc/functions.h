#ifndef FUNCTIONS_H
#define FUNCITONS_H

#include <Arduino.h>

uint8_t setCellCount(float voltage);
float calcAverage(float alpha, float oldValue, float newValue);

#endif