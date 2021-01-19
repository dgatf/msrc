#ifndef ESCPWM_H
#define ESCPWM_H

#define COMP_TO_MS(SCALER) (SCALER * 1000.0 / F_CPU)
#define MS_TO_COMP(SCALER) (F_CPU / (SCALER * 1000UL))

#include <Arduino.h>
#include "device.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
extern void (*TIMER1_CAPT_handlerP)();
extern void (*TIMER1_OVF_handlerP)();
#endif
#if defined(__AVR_ATmega2560__)
extern void (*TIMER4_CAPT_handlerP)();
extern void (*TIMER4_OVF_handlerP)();
#endif

class EscPWM : public AbstractDevice
{
private:
    static volatile uint16_t escPwmDuration;
    static volatile bool escPwmRunning;
    static volatile bool escPwmUpdate;
    uint8_t alphaRpm_;
    float rpm_;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
    static void TIMER1_CAPT_handler();
    static void TIMER1_OVF_handler();
#endif
#if defined(__AVR_ATmega2560__)
    static void TIMER4_CAPT_handler();
    static void TIMER4_OVF_handler();
#endif

protected:
public:
    EscPWM(uint8_t alphaRpm);
    void begin();
    float read(uint8_t index);
};

#endif