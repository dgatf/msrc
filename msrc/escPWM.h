#ifndef ESCPWM_H
#define ESCPWM_H

#define PWM_COMP_TO_MS(SCALER) (SCALER * 1000.0 / F_CPU)
#define PWM_MS_TO_COMP(SCALER) (F_CPU / (SCALER * 1000UL))

#include <Arduino.h>
#include "device.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
extern void (*TIMER1_CAPT_handlerP)();
extern void (*TIMER1_COMPB_handlerP)();
#endif
#if defined(__AVR_ATmega2560__)
extern void (*TIMER4_CAPT_handlerP)();
extern void (*TIMER4_COMPB_handlerP)();
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
extern void (*FTM0_IRQ_handlerP)();
#endif

class EscPWM : public AbstractDevice
{
private:
    static volatile uint16_t escPwmDuration;
    static volatile bool escPwmRunning, escPwmUpdate;
    uint8_t alphaRpm_;
    float rpm_;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
    static void TIMER1_CAPT_handler();
    static void TIMER1_COMPB_handler();
#endif
#if defined(__AVR_ATmega2560__)
    static void TIMER4_CAPT_handler();
    static void TIMER4_COMPB_handler();
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    static void FTM0_IRQ_handler();
#endif

protected:
public:
    EscPWM(uint8_t alphaRpm);
    void begin();
    virtual void update();
    float *rpmP();
};

#endif