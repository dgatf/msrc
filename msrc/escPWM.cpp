#include "escPWM.h"

volatile uint16_t escPwmDuration = 0;
volatile bool escPwmRunning = false;
volatile bool escPwmUpdate = false;

EscPWM::EscPWM(uint8_t alphaRpm) : alphaRpm_(alphaRpm) {}

void EscPWM::TIMER1_CAPT_handler()
{
    escPwmDuration = ICR1;
    TCNT1 = 0;              // reset timer
    escPwmRunning = true;
    escPwmUpdate = true;
}

void EscPWM::TIMER1_OVF_handler()
{
    escPwmRunning = false;
}

void EscPWM::begin()
{
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_OVF_handlerP = TIMER1_OVF_handler;
    // TIMER1 setup
    TCCR1A = 0;                                     // normal mode
    TCCR1B = _BV(CS11) | _BV(ICES1) | _BV(ICNC1);   // scaler 8 | capture rising | filter
    TIMSK1 = _BV(ICIE1) | _BV(TOIE1);               // capture interrupt (ICP1 = PIN 8) | overflow interrupt

}

float EscPWM::read(uint8_t index)
{
    if (index == 0)
    {
        noInterrupts();
        if (escPwmRunning) { 
            if (escPwmUpdate) {
                float rpm = 60000000UL / (escPwmDuration * COMP_TO_MICROS);
                rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
                escPwmUpdate = false;
            }
        }
        else {
            rpm_ = 0;
        }
        interrupts();
#ifdef DEBUG_ESC
        Serial.print("RPM: ");
        Serial.println(rpm_);
#endif
#ifdef SIM_SENSORS
        return 10000;
#endif
        return rpm_;
    }
    return 0;
}
