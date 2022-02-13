#ifndef ESCAPDHV_H
#define ESCAPDHV_H

#define APDHV_ESCSERIAL_TIMEOUT 3
#define APDHV_PACKET_LENGHT 22

#include <Arduino.h>
#include "device.h"

class EscApdHV : public AbstractDevice, Consumption
{
private:
    AbstractSerial &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_, cellCount_ = 255;
    float rpm_ = 0, voltage_ = 0, current_ = 0, temp_ = 0, cellVoltage_ = 0, consumption_ = 0;

    float calcTemp(uint16_t rawVal);

protected:
public:
    EscApdHV(AbstractSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp);
    void begin();
    void update();
    uint16_t get_crc16(uint8_t *buffer);
    float *rpmP();
    float *voltageP();
    float *currentP();
    float *tempP();
    float *consumptionP();
    float *cellVoltageP();

};

#endif