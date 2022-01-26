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

/* ESCHW4_DIVISOR and ESCHW4_AMPGAIN values

Divisor: Cells range

3-6S (LV): divisor = 11 
3-8S (LV v2): divisor = 15.4
5-12s (HV): divisor = 21

Gain: Amperage

60A: gain = 6
80A: gain = 7.8
100A: gain = 9(1)
120A: gain = 10
130A: gain = 11.3(1)
150A: gain = 12.9(1)
160A: gain = 13.7(1)
200A: gain = 16.9

(1) Extrapolated from confirmed models

*/

#define ESCHW4_DIVISOR 11
#define ESCHW4_AMPGAIN 10
#define ESCHW4_CURRENT_MAX 250

#include <Arduino.h>
#include "device.h"

class EscHW4 : public AbstractDevice
{
private:
    int16_t rawCurrentOffset_ = -1;
    AbstractSerial &serial_;
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_, type_, cellCount_ = 255;
    uint16_t thr_ = 0, pwm_ = 0;
    float rpm_ = 0, consumption_ = 0, voltage_ = 0, current_ = 0, tempFet_ = 0, tempBec_ = 0, cellVoltage_ = 0;
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
    EscHW4(AbstractSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp, uint8_t type);
    void begin();
    void update();
    float *rpmP();
    float *consumptionP();
    float *voltageP();
    float *currentP();
    float *tempFetP();
    float *tempBecP();
    float *cellVoltageP();
#ifdef ESC_SIGNATURE
    float *signatureP();
#endif
};

#endif