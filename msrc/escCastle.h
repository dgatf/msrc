#ifndef ESCCASTLE_H
#define ESCCASTLE_H

/* castleTelemetry:
    index   element                          scaler
    0       calib 1 (1000us)                 0
    1       Volt (V)                         20
    2       rippleVolt (V)                   4
    3       Curr (A)                         50
    4       Thr (0.1-0.2 ms esc pulse)       1
    5       Output power (%)                 0.2502
    6       rpm                              20416.7
    7       becVolt (V)                      4
    8       becCurr (A)                      4
    9       temp (C) or calib 2 (500us)      30
    10      temp ntc (C) or calib 2 (500us)  63.8125
*/

//#define DEBUG_ESC

#define FIXED_CALIB
//#define DEBUG_CALIB

#define MS_TO_COMP(SCALER) F_CPU / (SCALER * 1000UL)
#define RX_MAX_CYCLES 2  // minimum is 2

#define CASTLE_VOLTAGE 1
#define CASTLE_RIPPLE_VOLTAGE 2
#define CASTLE_CURRENT 3
#define CASTLE_RPM 6
#define CASTLE_BEC_VOLTAGE 7
#define CASTLE_BEC_CURRENT 8
#define CASTLE_TEMP 9
#define CASTLE_TEMP_NTC 10
#define CASTLE_CELL_VOLTAGE 11

#define R0 10000.0F
#define R2 10200.0F
#define B 3455.0F

#include <Arduino.h>
#include "device.h"
#include "escCell.h"

extern void (*TIMER1_CAPT_handlerP)();
extern void (*TIMER1_COMPB_handlerP)();
extern void (*INT0_handlerP)();
extern void (*TIMER2_COMPA_handlerP)();

class EscCastle : public AbstractDevice, public EscCell
{
private:
    const float scaler[11] = {0, 20, 4, 50, 1, 0.2502, 20416.7, 4, 4, 30, 63.8125};
    uint8_t alphaRpm_, alphaVolt_, alphaCurr_, alphaTemp_;
    static void TIMER1_CAPT_handler();
    static void TIMER1_COMPB_handler();
    static void INT0_handler();
    static void TIMER2_COMPA_handler();

protected:
public:
    EscCastle(uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp);
    void begin();
    float read(uint8_t index);
};

#endif