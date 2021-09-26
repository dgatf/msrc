#ifndef ESCHW4_H
#define ESCHW4_H

#define ESCHW4_ESCSERIAL_TIMEOUT 5000
#define ESCHWV4_PACKET_LENGHT 19
#define ESCHWV4_SIGNATURE_LENGHT 13

#define ESCHW4_NTC_BETA 3950.0
#define ESCHW4_NTC_R1 10000.0
#define ESCHW4_NTC_R_REF 47000.0
#define ESCHW4_DIFFAMP_SHUNT (0.25 / 1000)
#define ESCHW4_V_REF 3.3
#define ESCHW4_ADC_RES 4096.0

#include <Arduino.h>
#include "device.h"
#include "escCell.h"

class EscHW4 : public AbstractDevice, public EscCell
{
private:
    const uint8_t voltageDivisor_[4] = {11, 21, 11, 21};
    const float ampGain_[4] = {10, 16.9, 10, 16.9};
    int16_t rawCurrentOffset_ = -1;
    Stream &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_, type_;
    uint16_t thr_ = 0, pwm_ = 0;
    float rpm_ = 0, voltage_ = 0, current_ = 0, tempFet_ = 0, tempBec_ = 0;
#ifdef ESC_SIGNATURE
#ifdef SIM_SENSORS
    uint8_t signature_[12] = {0x03, 0xE8, 0x01, 0x08, 0x5B, 0x00, 0x01, 0x00, 0x21, 0x21, 0x10, 0x20};
#else
    uint8_t signature_[12] = {0};
#endif
#endif
    float calcVolt(uint16_t voltRaw);
    float calcTemp(uint16_t tempRaw);
    float calcCurr(uint16_t currentRaw);

protected:
public:
    EscHW4(Stream &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp, uint8_t type);
    virtual void update();
    float *rpmP();
    float *voltageP();
    float *currentP();
    float *tempFetP();
    float *tempBecP();
#ifdef ESC_SIGNATURE
    float *signatureP();
#endif
};

#endif