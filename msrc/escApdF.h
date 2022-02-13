#ifndef ESCAPDF_H
#define ESCAPDF_H

#define APDF_ESCSERIAL_TIMEOUT 3
#define APDF_PACKET_LENGHT 12
#define KISS_PACKET_LENGHT 10

#include <Arduino.h>
#include "device.h"

class EscApdF : public AbstractDevice, Consumption
{
private:
    AbstractSerial &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_, cellCount_ = 255;
    float rpm_ = 0, voltage_ = 0, current_ = 0, consumption_ = 0, temp_ = 0, cellVoltage_ = 0;

protected:
public:
    EscApdF(AbstractSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp);
    void begin();
    void update();
    uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
    uint8_t get_crc8(uint8_t *buffer, uint8_t lenght);
    float *rpmP();
    float *voltageP();
    float *currentP();
    float *tempP();
    float *consumptionP();
    float *cellVoltageP();
};

#endif