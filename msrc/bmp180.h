#ifndef BMP180_H
#define BMP180_H

#define BMP180_ADDR 0x77

#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

#define BMP180_ULTRALOWPOWER 0
#define BMP180_STANDARD 1
#define BMP180_HIGHRES 2
#define BMP180_ULTRAHIGHRES 3

#define BMP180_STATE_READY 0
#define BMP180_STATE_WAIT_TEMP 1
#define BMP180_STATE_WAIT_PRESS 2

#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "bmp.h"

class Bmp180Interface : public Bmp
{
private:
    int16_t AC1_, AC2_, AC3_, VB1_, VB2_, MB_, MC_, MD_;
    uint16_t AC4_, AC5_, AC6_, timestamp_ = 0;
    float c5_, c6_, mc_, md_, x0_, x1_, x2_, y0_, y1_, y2_, p0_, p1_, p2_;
    uint8_t alphaTemp_, alphaDef_, address_, sampling_ = 3, state_ = 0, delay_ = 0;
    uint8_t startTemperature();
    uint8_t startPressure();
    bool readTemperature();
    bool readPressure();

public:
    Bmp180Interface(uint8_t alphaTemp, uint8_t alphaDef, uint8_t address);
    bool begin();
    float read(uint8_t index);
};

#endif