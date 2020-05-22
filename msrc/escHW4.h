#ifndef ESCHW4_H
#define ESCHW4_H

//#define DEBUG_ESC
//#define DEBUG_SIGNATURE

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
    const uint8_t signature_[4][12] = {{0x9B, 0x03, 0xE8, 0x01, 0x08, 0x5B, 0x00, 0x01, 0x00, 0x21, 0x21, 0xB9},      // V4LV60A - ESCHW4_TYPE_V4_LV
                                       {0x9B, 0x03, 0xE8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x21, 0xB9},      // BYTES 5-9? ESCHW4_TYPE_V4_HV
                                       {0x9B, 0x03, 0xE8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x21, 0xB9},      // BYTES 5-9? ESCHW4_TYPE_V5_LV
                                       {0x9B, 0x03, 0xE8, 0x01, 0x0B, 0x41, 0x21, 0x44, 0xB9, 0x21, 0x21, 0xB9}};     // V5HV130A - ESCHW4_TYPE_V5_HV
    const float voltageDivisor_[5] = {11, 21, 11, 21, 11};
    const float rawCurrentOffset_[5] = {15, 15, 660, 660, 15};
    HardwareSerial &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_;
    float value_[8] = {0};
    uint8_t type_ = ESCHW4_TYPE_V5_HV + 1;

    float calcVolt(uint16_t voltRaw);
    float calcTemp(uint16_t tempRaw);
    float calcCurr(uint16_t currentRaw);

protected:
public:
    EscHW4Interface(HardwareSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp);
    void begin();
    bool update();
    float read(uint8_t index);
};

#endif