#include "pwmout.h"

PwmOut::PwmOut() {}

bool PwmOut::isEnabled_ = false;

float *PwmOut::rpmP_ = NULL;

void PwmOut::enable()
{
    isEnabled_ = true;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
    // TIMER1: MODE 15 (TOP OCRA), SCALER 8. OC1B, PB2, PIN 10
    DDRB |= _BV(DDB2);
    TCCR1A = _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
#endif
#if defined(__AVR_ATmega2560__)
    // TIMER4: MODE 15 (TOP OCRA), SCALER 8. OC4B, PH4, PIN 7
    DDRH |= _BV(DDH4);
    TCCR4A = _BV(WGM41) | _BV(WGM40);
    TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);
#endif
#if defined(__AVR_ATmega32U4__)
    // TIMER1: MODE 15 (TOP OCRA), SCALER 8. OC1B, PB6
    DDRB |= _BV(DDB6);
    TCCR1A = _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    FTM0_SC = 0;
    delayMicroseconds(1);
    FTM0_CNT = 0;
    SIM_SCGC6 |= SIM_SCGC6_FTM0;   // ENABLE CLOCK
    FTM0_SC = FTM_SC_PS(7);        // PRESCALER 128
    FTM0_SC |= FTM_SC_CLKS(1);     // ENABLE COUNTER
    FTM0_C0SC = 0;                 // DISABLE CHANNEL
    delayMicroseconds(1);          //
    FTM0_C0SC = FTM_CSC_ELSB;      // HIGH PULSES
    FTM0_C0SC |= FTM_CSC_MSB;      // OUTPUT PWM
    PORTC_PCR1 |= PORT_PCR_MUX(4); // TPM0_CH0 MUX 4 -> PTC1 -> 22/A8
#endif
}

void PwmOut::stop()
{
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB) || defined(__AVR_ATmega32U4__)
    TCCR1A &= ~_BV(COM1B1);
#endif
#if defined(__AVR_ATmega2560__)
    TCCR4A &= ~_BV(COM4B1);
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    SIM_SCGC6 &= ~SIM_SCGC6_FTM0; // DISABLE CLOCK
    FTM0_CNT = 0;
#endif
}

void PwmOut::disable()
{
    isEnabled_ = false;
    stop();
}

void PwmOut::update()
{
    static float rpm = 0;
    if (isEnabled_ && rpmP_ != NULL)
    {
        if (*rpmP_ != rpm)
        {
            rpm = *rpmP_;
            noInterrupts();
            if (rpm >= 2000)
            {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB) || defined(__AVR_ATmega32U4__)
                TCCR1A |= _BV(COM1B1);
                OCR1A = (60000 / rpm) * MS_TO_COMP(8) - 1;
                OCR1B = PWMOUT_DUTY * OCR1A;
#endif
#if defined(__AVR_ATmega2560__)
                TCCR4A |= _BV(COM4B1);
                OCR4A = (60000 / rpm) * MS_TO_COMP(8) - 1;
                OCR4B = PWMOUT_DUTY * OCR4A;
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
                SIM_SCGC6 |= SIM_SCGC6_FTM0;                    // ENABLE CLOCK
                FTM0_MOD = (60000 / rpm) * MS_TO_COMP(128) - 1; // SET FRECUENCY
                FTM0_C0V = PWMOUT_DUTY * FTM0_MOD;              // SET DUTY
#endif
            }
            else
            {
                stop();
            }
            interrupts();
        }
    }
}

void PwmOut::setRpmP(float *rpmP)
{
    rpmP_ = rpmP;
}
