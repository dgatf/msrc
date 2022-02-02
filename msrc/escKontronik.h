#ifndef ESCKONTRONIK_H
#define ESCKONTRONIK_H

#define KONTRONIK_ESCSERIAL_TIMEOUT 2000
#define KONTRONIK_PACKET_LENGHT 35

#include <Arduino.h>
#include "device.h"

class EscKontronik : public AbstractDevice
{
private:
    AbstractSerial &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_, cellCount_ = 255;
    float rpm_ = 0, consumption_ = 0, voltage_ = 0, current_ = 0, becCurrent_ = 0, becVoltage_ = 0, tempFet_ = 0, tempBec_ = 0, cellVoltage_ = 0;

protected:
public:
    EscKontronik(AbstractSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp);
    void begin();
    void update();
    float *rpmP();
    float *consumptionP();
    float *voltageP();
    float *currentP();
    float *tempFetP();
    float *tempBecP();
    float *becVoltageP();
    float *becCurrentP();
    float *cellVoltageP();
};

#endif