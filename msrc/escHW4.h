#ifndef ESCHW4_H
#define ESCHW4_H

#define ESCHW4_TYPE_V4_LV 0
#define ESCHW4_TYPE_V4_HV 1
#define ESCHW4_TYPE_V5_LV 2
#define ESCHW4_TYPE_V5_HV 3

#define ESCHW4_THR 0
#define ESCHW4_PWM 1
#define ESCHW4_RPM 2
#define ESCHW4_VOLTAGE 3
#define ESCHW4_CURRENT 4
#define ESCHW4_TEMPFET 5
#define ESCHW4_TEMPBEC 6
#define ESCHW4_CELL_VOLTAGE 7

#define ESCHW4_ESCSERIAL_TIMEOUT 3

#define ESCHW4_NTC_BETA 3950.0
#define ESCHW4_NTC_R1 10000.0
#define ESCHW4_NTC_R_REF 47000.0
#define ESCHW4_DIFFAMP_GAIN 13.6
#define ESCHW4_DIFFAMP_SHUNT 0.25 / 1000
#define ESCHW4_V_REF 3.3
#define ESCHW4_ADC_RES 4096.0

#include <Arduino.h>
#include "device.h"
#include "escCell.h"

class EscHW4Interface : public AbstractDevice, public EscCell
{
private:
    const float voltageDivisor_[4] = {11, 21, 11, 21};
    const float rawCurrentOffset_[4] = {15, 15, 660, 660};
    HardwareSerial &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_;
    float value_[7] = {0};
    uint8_t type_;

    float calcVolt(uint16_t voltRaw);
    float calcTemp(uint16_t tempRaw);
    float calcCurr(uint16_t currentRaw);

protected:
public:
    EscHW4Interface(HardwareSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp, uint8_t type);
    void begin();
    bool update();
    float read(uint8_t index);
};

#endif