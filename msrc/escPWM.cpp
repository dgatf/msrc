#include "escPWM.h"

volatile uint16_t EscPWM::escPwmDuration = 0;
volatile bool EscPWM::escPwmRunning = false;
volatile bool EscPWM::escPwmUpdate = false;

EscPWM::EscPWM(uint8_t alphaRpm) : alphaRpm_(alphaRpm) {}

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
void EscPWM::TIMER1_CAPT_handler()
{
    escPwmDuration = ICR1;
    TCNT1 = 0; // reset timer
    escPwmRunning = true;
    escPwmUpdate = true;
}

void EscPWM::TIMER1_OVF_handler()
{
    escPwmRunning = false;
}
#endif

void EscPWM::begin()
{
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
    // TIMER1: MODE 0 (NORMAL), SCALER 8, CAPTURE AND OVERFLOW INTERRUPT. ICP1, PB0, PIN 8
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_OVF_handlerP = TIMER1_OVF_handler;
    TCCR1A = 0;
    TCCR1B = _BV(CS11) | _BV(ICES1) | _BV(ICNC1);
    TIMSK1 = _BV(ICIE1) | _BV(TOIE1);
#endif
}

float EscPWM::read(uint8_t index)
{
    if (index == 0)
    {
        noInterrupts();
        if (escPwmRunning)
        {
            if (escPwmUpdate)
            {
                float rpm = 60000UL / (escPwmDuration * COMP_TO_MS(8));
                rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
                escPwmUpdate = false;
            }
        }
        else
        {
            rpm_ = 0;
        }
        interrupts();
#ifdef DEBUG_ESC
        DEBUG_SERIAL.print("RPM: ");
        DEBUG_SERIAL.println(rpm_);
#endif
#ifdef SIM_SENSORS
        return 10000;
#endif
        return rpm_;
    }
    return 0;
}
