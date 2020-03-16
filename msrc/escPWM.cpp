#include "escPWM.h"

// Pwm in: TIMER 1 CAPT, PIN 8
volatile uint16_t pwmInLenght = 60000;
volatile uint32_t tsPwmIn = 0;

ISR(TIMER1_CAPT_vect)
{
    volatile static uint16_t pwmInInit = 0;
    if (ICR1 - pwmInInit > 0)
    {
        pwmInLenght = ICR1 - pwmInInit;
        pwmInInit = ICR1;
        tsPwmIn = micros();
    }
}

EscPWMInterface::EscPWMInterface(uint8_t alphaRpm) : alphaRpm_(alphaRpm) {}

void EscPWMInterface::begin() {
    // TIMER1,capture ext int, scaler 8. PIN 8
    TCCR1A = 0;
    TCCR1B = _BV(CS11);
    TIMSK1 = _BV(ICIE1);
}

float EscPWMInterface::read(uint8_t index)
{
    if (index == 0) {
        static uint8_t cont = 0;
        float rpm;
        if (pwmInLenght > 0 &&
            pwmInLenght * COMP_TO_MICROS < PWM_IN_TRIGGER_MICROS &&
            micros() - tsPwmIn < PWM_IN_TRIGGER_MICROS)
        {
            if (cont > PWM_IN_TRIGGER_PULSES)
            {
                rpm = 60000000UL / pwmInLenght * COMP_TO_MICROS;
            }
            if (cont <= PWM_IN_TRIGGER_PULSES)
                cont++;
    #ifdef DEBUG_ESC
            Serial.print("RPM: ");
            Serial.println(rpm_);
    #endif
        }
        else
        {
            rpm_ = 0;
            cont = 0;
        }
        rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
    #ifdef SIM_SENSORS
        return 10000;
    #endif
        return rpm_;
    }
    return 0;
}
