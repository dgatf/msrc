#include "escPWM.h"

volatile uint16_t EscPWM::escPwmDuration = 0;
volatile bool EscPWM::escPwmRunning = false;
volatile bool EscPWM::escPwmUpdate = false;

EscPWM::EscPWM(uint8_t alphaRpm) : alphaRpm_(alphaRpm) {}

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
void EscPWM::TIMER1_CAPT_handler()
{
    escPwmDuration = ICR1;
    TCNT1 = 0; // reset timer
    escPwmRunning = true;
    escPwmUpdate = true;
#ifdef DEBUG_ESC
    DEBUG_SERIAL.println(escPwmDuration);
#endif
}

void EscPWM::TIMER1_OVF_handler()
{
    escPwmRunning = false;
#ifdef DEBUG_ESC
    DEBUG_SERIAL.println("STOP");
#endif
}
#endif

#if defined(__AVR_ATmega2560__)
void EscPWM::TIMER4_CAPT_handler()
{
    escPwmDuration = ICR4;
    TCNT4 = 0; // reset timer
    escPwmRunning = true;
    escPwmUpdate = true;
#ifdef DEBUG_ESC
    DEBUG_SERIAL.println(escPwmDuration);
#endif
}

void EscPWM::TIMER4_OVF_handler()
{
    escPwmRunning = false;
#ifdef DEBUG_ESC
    DEBUG_SERIAL.println("STOP");
#endif
}
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
void EscPWM::FTM0_IRQ_handler()
{
    if (FTM0_C4SC & FTM_CSC_CHF) // TIMER CAPTURE INTERRUPT CH4
    {
        escPwmDuration = FTM0_CNT;
        FTM0_C4SC |= FTM_CSC_CHF; // CLEAR FLAG
        FTM0_CNT = 0;             // RESET COUNTER
        escPwmRunning = true;
        escPwmUpdate = true;
#ifdef DEBUG_ESC
        DEBUG_SERIAL.println(escPwmDuration);
#endif
    }
    if (FTM0_SC & FTM_SC_TOF) // TIMER OVERFLOW INTERRUPT
    {
        FTM0_SC |= FTM_SC_TOF; // CLEAR FLAG
        escPwmRunning = false;
#ifdef DEBUG_ESC
        DEBUG_SERIAL.println("STOP");
#endif
    }
}
#endif

void EscPWM::begin()
{
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
    // TIMER1: MODE 0 (NORMAL), SCALER 8, CAPTURE AND OVERFLOW INTERRUPT. ICP1, PB0, PIN 8
    PORTB |= _BV(PB0); // PULL UP
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_OVF_handlerP = TIMER1_OVF_handler;
    TCCR1A = 0;
    TCCR1B = _BV(CS11) | _BV(ICES1) | _BV(ICNC1);
    TIMSK1 = _BV(ICIE1) | _BV(TOIE1);
#endif

#if defined(__AVR_ATmega2560__)
    // TIMER4: MODE 0 (NORMAL), SCALER 8, CAPTURE AND OVERFLOW INTERRUPT. ICP4, PL0, PIN 49
    PORTL |= _BV(PL0); // PULL UP
    TIMER4_CAPT_handlerP = TIMER4_CAPT_handler;
    TIMER4_OVF_handlerP = TIMER4_OVF_handler;
    TCCR4A = 0;
    TCCR4B = _BV(CS41) | _BV(ICES4) | _BV(ICNC4);
    TIMSK4 = _BV(ICIE4) | _BV(TOIE4);
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    FTM0_IRQ_handlerP = FTM0_IRQ_handler;
    FTM0_SC = 0;          // DISABLE TIMER
    delayMicroseconds(1); //
    FTM0_CNT = 0;
    SIM_SCGC6 |= SIM_SCGC6_FTM0; // ENABLE CLOCK
    FTM0_MOD = 0xFFFF;
    FTM0_SC = FTM_SC_PS(7);                     // PRESCALER 128
    FTM0_SC |= FTM_SC_CLKS(1);                  // ENABLE COUNTER
    FTM0_SC |= FTM_SC_TOIE;                     // ENABLE OVERFLOW INTERRUPT
    FTM0_C4SC = 0;                              // DISABLE CHANNEL
    delayMicroseconds(1);                       //
    FTM0_C4SC = FTM_CSC_ELSA;                   // CAPTURE RISING CH4
    FTM0_C4SC |= FTM_CSC_CHIE;                  // ENABLE INTERRUPT CH4
    PORTD_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_PE; // TPM0_CH4 MUX 4 -> PTD4 -> 6
    NVIC_ENABLE_IRQ(IRQ_FTM0);
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
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
                float rpm = 60000UL / (escPwmDuration * COMP_TO_MS(128));
#else
                float rpm = 60000UL / (escPwmDuration * COMP_TO_MS(8));
#endif
                rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
                escPwmUpdate = false;
            }
        }
        else
        {
            rpm_ = 0;
        }
        interrupts();
#ifdef SIM_SENSORS
        return 10000;
#endif
        return rpm_;
    }
    return 0;
}
