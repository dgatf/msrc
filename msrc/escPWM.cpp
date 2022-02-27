#include "escPWM.h"

volatile uint32_t EscPWM::escPwmDuration = 0;
volatile bool EscPWM::escPwmRunning = false;
volatile bool EscPWM::escPwmUpdate = false;
volatile uint8_t EscPWM::cycles_ = 0;

EscPWM::EscPWM(uint8_t alphaRpm) : alphaRpm_(alphaRpm) {}

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
void EscPWM::TIMER1_CAPT_handler()
{
    static uint16_t ts = 0;
    if (escPwmRunning)
        escPwmDuration = (ICR1 - ts) + (cycles_ * 65536UL);
    ts = ICR1;
    OCR1B = TCNT1;
    TIFR1 |= _BV(OCF1B);   // CLEAR TIMER1 OCRB CAPTURE FLAG
    TIMSK1 |= _BV(OCIE1B); // ENABLE TIMER1 OCRB INTERRUPT
    escPwmRunning = true;
    escPwmUpdate = true;
    cycles_ = 0;
#ifdef DEBUG_PWM
    //DEBUG_PRINT(escPwmDuration);
    DEBUG_PRINT((float)(escPwmDuration * COMP_TO_MS(1)));
    DEBUG_PRINT(" ");
    DEBUG_PRINTLN();
#endif
}

void EscPWM::TIMER1_COMPB_handler()
{
    cycles_++;
    if (cycles_ > ESCPWM_MAX_CYCLES)
    {
        escPwmRunning = false;
        escPwmDuration = 0xFFFFFFFF;
        TIMSK1 ^= _BV(OCIE1B); // DISABLE TIMER1 OCRA INTERRUPT
#ifdef DEBUG_PWM
        DEBUG_PRINT("X");
        DEBUG_PRINTLN();
#endif
    }
}
#endif

#if defined(__AVR_ATmega2560__)
void EscPWM::TIMER4_CAPT_handler()
{
    static uint16_t ts = 0;
    if (escPwmRunning)
        escPwmDuration = (ICR4 - ts) + (cycles_ * 65536UL);
    ts = ICR4;
    OCR4B = TCNT4;
    TIFR4 |= _BV(OCF4B);   // CLEAR TIMER4 OCRB CAPTURE FLAG
    TIMSK4 |= _BV(OCIE4B); // ENABLE TIMER4 OCRB INTERRUPT
    escPwmRunning = true;
    escPwmUpdate = true;
    cycles_ = 0;
#ifdef DEBUG_PWM
    //DEBUG_PRINT(escPwmDuration);
    DEBUG_PRINT((float)(escPwmDuration * COMP_TO_MS(1)));
    DEBUG_PRINT(" ");
    DEBUG_PRINTLN();
#endif
}

void EscPWM::TIMER4_COMPB_handler()
{
    cycles_++;
    if (cycles_ > ESCPWM_MAX_CYCLES)
    {
        escPwmRunning = false;
        escPwmDuration = 0xFFFFFFFF;
        TIMSK4 ^= _BV(OCIE4B); // DISABLE TIMER4 OCRA INTERRUPT
#ifdef DEBUG_PWM
        DEBUG_PRINT("X");
        DEBUG_PRINTLN();
#endif
    }
}
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
void EscPWM::FTM0_IRQ_handler()
{
    if (FTM0_C4SC & FTM_CSC_CHF) // TIMER CAPTURE INTERRUPT CH4
    {
        static uint16_t ts = 0;
        if (escPwmRunning)
            escPwmDuration = (uint16_t)(FTM0_CNT - ts) + (cycles_ * 65536UL);
        ts = FTM0_CNT;
        FTM0_C4SC |= FTM_CSC_CHF; // CLEAR FLAG
        FTM0_C0V = FTM0_CNT;
        FTM0_C0SC |= FTM_CSC_CHF;  // CLEAR FLAG
        FTM0_C0SC |= FTM_CSC_CHIE; // ENABLE CH0 INTERRUPT
        escPwmUpdate = true;
        escPwmRunning = true;
        cycles_ = 0;
#ifdef DEBUG_PWM
        //DEBUG_PRINT(escPwmDuration);
        DEBUG_PRINT((float)(escPwmDuration * COMP_TO_MS(128)));
        DEBUG_PRINT(" ");
        DEBUG_PRINTLN();
#endif
    }
    if (FTM0_C0SC & FTM_CSC_CHF) // TIMER CAPTURE INTERRUPT CH4
    {
        cycles_++;
        if (cycles_ > ESCPWM_MAX_CYCLES)
        {
            escPwmRunning = false;
            escPwmDuration = 0xFFFFFFFF;
            FTM0_C0SC &= ~FTM_CSC_CHIE; // DISABLE INTERRUPT
            FTM0_C0SC |= FTM_CSC_CHF;   // CLEAR FLAG
#ifdef DEBUG_PWM
            DEBUG_PRINT("X");
            DEBUG_PRINTLN();
#endif
        }
    }
}
#endif

void EscPWM::begin()
{
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
    // TIMER1: MODE 0 (NORMAL), SCALER 1, CAPTURE AND OVERFLOW INTERRUPT. ICP1, PB0, PIN 8
    PORTB |= _BV(PB0); // PULL UP
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_COMPB_handlerP = TIMER1_COMPB_handler;
    TCCR1A = 0;
    TCCR1B = _BV(CS10) | _BV(ICES1) | _BV(ICNC1); // SCALER 1
    TIMSK1 = _BV(ICIE1) | _BV(TOIE1);
#endif

#if defined(__AVR_ATmega2560__)
    // TIMER4: MODE 0 (NORMAL), SCALER 1, CAPTURE AND OVERFLOW INTERRUPT. ICP4, PL0, PIN 49
    PORTL |= _BV(PL0); // PULL UP
    TIMER4_CAPT_handlerP = TIMER4_CAPT_handler;
    TIMER4_COMPB_handlerP = TIMER4_COMPB_handler;
    TCCR4A = 0;
    TCCR4B = _BV(CS40) | _BV(ICES4) | _BV(ICNC4);
    TIMSK4 = _BV(ICIE4) | _BV(TOIE4);
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    FTM0_IRQ_handlerP = FTM0_IRQ_handler;
    FTM0_SC = 0;          // DISABLE TIMER
    delayMicroseconds(1); //
    FTM0_CNT = 0;
    SIM_SCGC6 |= SIM_SCGC6_FTM0; // ENABLE CLOCK
    FTM0_MOD = 0xFFFF;
    FTM0_SC = FTM_SC_PS(7);    // PRESCALER 128
    FTM0_SC |= FTM_SC_CLKS(1); // ENABLE COUNTER

    FTM0_C4SC = 0;                              // DISABLE CHANNEL
    delayMicroseconds(1);                       //
    FTM0_C4SC = FTM_CSC_ELSA;                   // CAPTURE RISING CH4
    FTM0_C4SC |= FTM_CSC_CHIE;                  // ENABLE INTERRUPT CH4
    PORTD_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_PE; // TPM0_CH4 MUX 4 -> PTD4 -> 6

    FTM0_C0SC = 0;           // DISABLE CHANNEL
    delayMicroseconds(1);    //
    FTM0_C0SC = FTM_CSC_MSA; // SOFTWARE CH0

    NVIC_ENABLE_IRQ(IRQ_FTM0);

#endif

    /*
#if defined(__IMXRT1062__)
    
    GPT1_CR = 0;                    // DISABLE TIMER (EN=0)
    delayMicroseconds(1);
    GPT1_CNT = 0;
    GPT1_CR |= GPT_CR_CLKSRC(B001); // CLOCK

    GPT1_PR = GPT_PR_PRESCALER(32); // PRESCALER 32
    GPT1_CR |= GPT_PR_IM1(B01);     // CAPTURE RISING CH1
    GPT1_CR |= GPT_CR_EN;           // ENABLE TIMER
    //PORT_PCR_MUX
    NVIC_ENABLE_IRQ(IRQ_GPT1);
#endif
*/
}

void EscPWM::update()
{
    noInterrupts();
    if (escPwmRunning)
    {
        if (escPwmUpdate && escPwmDuration > 1)
        {
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
            float rpm = 60000.0 / (escPwmDuration * COMP_TO_MS(32));
#else
            float rpm = 60000.0 / (escPwmDuration * COMP_TO_MS(1));
#endif
            rpm *= RPM_MULTIPLIER;
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
    rpm_ = 12345.67;
#endif
}

float *EscPWM::rpmP()
{
    return &rpm_;
}
