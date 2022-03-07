#include "escCastle.h"

volatile bool EscCastle::castleTelemetryReceived = false;
#ifdef SIM_SENSORS
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
volatile uint16_t EscCastle::castleTelemetry[12] = {(uint16_t)MS_TO_COMP(32), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(1.5 * MS_TO_COMP(32)), (uint16_t)(0.5 * MS_TO_COMP(32)), (uint16_t)(0.5 * MS_TO_COMP(32))};
#else
volatile uint16_t EscCastle::castleTelemetry[12] = {(uint16_t)MS_TO_COMP(8), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(1.5 * MS_TO_COMP(8)), (uint16_t)(0.5 * MS_TO_COMP(8)), (uint16_t)(0.5 * MS_TO_COMP(8))};
#endif
#else
volatile uint16_t EscCastle::castleTelemetry[12] = {0};
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
volatile uint16_t EscCastle::castleCompsPerMilli = 1 * MS_TO_COMP(32);
#else
volatile uint16_t EscCastle::castleCompsPerMilli = 1 * MS_TO_COMP(8);
#endif
volatile uint8_t EscCastle::castleCont = 0;
volatile uint16_t EscCastle::castlePwmRx = 0;
volatile bool EscCastle::castleUpdated = true;

EscCastle::EscCastle(uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp) {}

#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)

volatile uint8_t EscCastle::castleRxLastReceived = 0;

void EscCastle::TIMER1_CAPT_handler() // RX INPUT
{
    static uint16_t ts = 0;
    if (TCCR1B & _BV(ICES1)) // RX RISING
    {
        ts = ICR1;
    }
    else // RX FALLING
    {
        if (ts < ICR1)
        {
            OCR1B = ICR1 - ts;
        }
        else
        {
            OCR1B = ICR1 + OCR1A - ts;
        }
        castlePwmRx = OCR1B;
        TIMSK1 |= _BV(OCIE1B);
        castleRxLastReceived = 0;
#ifdef DEBUG_CASTLE_RX
        DEBUG_PRINT(castlePwmRx);
        DEBUG_PRINTLN();
#endif
    }
    TCCR1B ^= _BV(ICES1); // TOGGLE ICP1 DIRECTION
}

void EscCastle::TIMER1_COMPB_handler() // START INPUT STATE
{
    DDRB &= ~_BV(DDB2);                    // PWM OUT (PB2, PIN10) INPUT
    PORTB |= _BV(PB2);                     // PB2 PULLUP
    EIFR |= _BV(INTF0);                    // CLEAR INT0 FLAG
    EIMSK = _BV(INT0);                     // ENABLE INT0 (PD2, PIN2)
    OCR2A = TCNT2 + 12 * MS_TO_COMP(1024); // 12ms AHEAD
    TIFR2 |= _BV(OCF2A);                   // CLEAR TIMER2 OCRA CAPTURE FLAG
    TIMSK2 |= _BV(OCIE2A);                 // ENABLE TIMER2 OCRA INTERRUPT
}

void EscCastle::INT0_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = TCNT1 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_PRINT(castleTelemetry[castleCont]);
    DEBUG_PRINT(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
        castleUpdated = true;
    }
}

void EscCastle::TIMER2_COMPA_handler() // START OUTPUT STATE
{
    EIMSK = 0;              // DISABLE INT0 (PD2, PIN2)
    TIMSK2 &= ~_BV(OCIE2A); // DISABLE TIMER2 OCRA INTERRUPT
    DDRB |= _BV(DDB2);      // PWM OUT PIN 10 OUTPUT
    if (castleRxLastReceived > RX_MAX_CYCLES)
    {
        OCR1B = 0;
        TIMSK1 &= ~_BV(OCIE1B); // DISABLE PIN
#ifdef DEBUG_CASTLE_RX
        DEBUG_PRINT("X");
        DEBUG_PRINTLN();
#endif
    }
    else
    {
        castleRxLastReceived++;
    }
    if (!castleTelemetryReceived)
    {
        //castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
        castleCont = 0;
#ifdef DEBUG_CASTLE
        DEBUG_PRINTLN();
        DEBUG_PRINT("1 ");
        DEBUG_PRINT(millis());
        DEBUG_PRINT(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
void EscCastle::TIMER1_CAPT_handler() // RX INPUT
{
    static uint16_t ts = 0;
    if (TCCR1B & _BV(ICES1)) // RX RISING (PULSE START)
    {
        ts = ICR1;
    }
    else // RX FALLING (PULSE END)
    {
        if (!(TIMSK4 & _BV(OCIE4B)))
        {
            TCNT4 = 0;             // RESET COUNTER
            DDRD |= _BV(DDD2);     // OUTPUT OC4B (PD2, 2)
            TIFR4 |= _BV(OCF4B);   // CLEAR OCRB FLAG
            TIMSK4 |= _BV(OCIE4B); // ENABLE OCRB MATCH INTERRUPT
            TIFR1 |= _BV(TOV1);    // CLEAR OVERFLOW FLAG
            TIMSK1 |= _BV(TOIE1);  // ENABLE OVERFLOW INTERRUPT
        }
        OCR4B = ICR1 - ts;
        castlePwmRx = OCR4B; // KEEP PWM STATE FOR TELEMETRY PULSE LENGHT
        TCNT1 = 0;           // RESET COUNTER
#ifdef DEBUG_CASTLE_RX
        DEBUG_PRINT(castlePwmRx);
        DEBUG_PRINTLN();
#endif
    }
    TCCR1B ^= _BV(ICES1); // TOGGLE ICP1 EDGE
}

void EscCastle::TIMER1_OVF_handler() // NO RX INPUT
{
    DDRD &= ~_BV(DDD2);    // INPUT OC4B (PD2, 2)
    PORTD |= _BV(PD2);     // PD2 PULLUP
    TIMSK4 = 0;            // DISABLE INTERRUPTS
    TCCR1B |= _BV(ICES1);  // RISING EDGE
    TIMSK1 &= ~_BV(TOIE1); // DISABLE OVERFLOW INTERRUPT
}

void EscCastle::TIMER4_COMPB_handler() // START INPUT STATE
{
    DDRD &= ~_BV(DDD2);   // INPUT OC4B (PD2, 2)
    PORTD |= _BV(PD2);    // PD2 PULLUP
    TIFR4 |= _BV(ICF4);   // CLEAR ICP4 CAPTURE FLAG
    TIMSK4 |= _BV(ICIE4); // ENABLE ICP4 CAPT

    TCNT3 = 0;             // RESET COUNTER
    TIFR3 |= _BV(OCF3A);   // CLEAR TIMER3 OCRA CAPTURE FLAG
    TIMSK3 |= _BV(OCIE3A); // ENABLE TIMER3 OCRA INTERRUPT
}

void EscCastle::TIMER4_CAPT_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = ICR4 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_PRINT(castleTelemetry[castleCont]);
    DEBUG_PRINT(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
        castleUpdated = true;
    }
}

void EscCastle::TIMER3_COMPA_handler() // START OUTPUT STATE
{
    TIMSK4 &= ~_BV(ICIE4);  // DISABLE ICP4 CAPT
    DDRD |= _BV(DDD2);      // OUTPUT OC4B (PD2, 2)
    TIMSK3 &= ~_BV(OCIE3A); // DISABLE TIMER3 OCRA INTERRUPT
    if (!castleTelemetryReceived)
    {
        castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
        castleCont = 0;
#ifdef DEBUG_CASTLE
        DEBUG_PRINTLN();
        DEBUG_PRINT("2 ");
        DEBUG_PRINT(millis());
        DEBUG_PRINT(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

#if defined(__AVR_ATmega2560__)
void EscCastle::TIMER4_CAPT_handler() // RX INPUT
{
    static uint16_t ts = 0;
    if (TCCR4B & _BV(ICES4)) // RX RISING (PULSE START)
    {
        ts = ICR4;
    }
    else // RX FALLING (PULSE END)
    {
        if (!(TIMSK5 & _BV(OCIE5B)))
        {
            TCNT5 = 0;             // RESET COUNTER
            DDRL |= _BV(DDL4);     // OUTPUT OC5B (PL4, 45)
            TIFR5 |= _BV(OCF5B);   // CLEAR OCRB CAPTURE FLAG
            TIMSK5 |= _BV(OCIE5B); // ENABLE OCRB MATCH INTERRUPT
            TIFR4 |= _BV(TOV4);    // CLEAR OVERFLOW FLAG
            TIMSK4 |= _BV(TOIE4);  // ENABLE OVERFLOW INTERRUPT
        }
        OCR5B = ICR4 - ts;
        castlePwmRx = OCR5B; // KEEP PWM STATE FOR TELEMETRY PULSE LENGHT
        TCNT4 = 0;           // RESET COUNTER
#ifdef DEBUG_CASTLE_RX
        DEBUG_PRINT(castlePwmRx);
        DEBUG_PRINTLN();
#endif
    }
    TCCR4B ^= _BV(ICES4); // TOGGLE ICP4 EDGE
}

void EscCastle::TIMER4_OVF_handler() // NO RX INPUT
{
    DDRL &= ~_BV(DDL4);    // INPUT OC5B (PL4, 45)
    PORTL |= _BV(PL4);     // PL4 PULLUP
    TIMSK5 = 0;            // DISABLE INTERRUPTS
    TCCR4B |= _BV(ICES4);  // RISING EDGE
    TIMSK4 &= ~_BV(TOIE4); // DISABLE OVERFLOW INTERRUPT
#ifdef DEBUG_CASTLE_RX
    DEBUG_PRINT("X");
    DEBUG_PRINTLN();
#endif
}

void EscCastle::TIMER5_COMPB_handler() // START INPUT STATE
{
    DDRL &= ~_BV(DDL4);                 // INPUT OC5B (PL4, 45)
    PORTL |= _BV(PL4);                  // PL4 PULLUP
    TIFR5 |= _BV(OCF5C) | _BV(ICF5);    // CLEAR ICP5 CAPTURE/OC5C MATCH FLAGS
    TIMSK5 |= _BV(OCIE5C) | _BV(ICIE5); // ENABLE ICP5 CAPT/OC5C MATCH
}

void EscCastle::TIMER5_CAPT_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = ICR5 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_PRINT(castleTelemetry[castleCont]);
    DEBUG_PRINT(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
        castleUpdated = true;
    }
}

void EscCastle::TIMER5_COMPC_handler() // START OUTPUT STATE
{
    TIMSK5 &= ~_BV(ICIE5); // DISABLE ICP5 CAPT
    DDRL |= _BV(DDL4);     // OUTPUT OC5B (PL4, 45)
    if (!castleTelemetryReceived)
    {
        castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
        castleCont = 0;
#ifdef DEBUG_CASTLE
        DEBUG_PRINTLN();
        DEBUG_PRINT("3 ");
        DEBUG_PRINT(millis());
        DEBUG_PRINT(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

#if defined(__AVR_ATmega32U4__)
void EscCastle::TIMER3_CAPT_handler() // RX INPUT
{
    static uint16_t ts = 0;
    if (TCCR3B & _BV(ICES3)) // RX RISING (PULSE START)
    {
        ts = ICR3;
    }
    else // RX FALLING (PULSE END)
    {
        if (!(TIMSK1 & _BV(OCIE1B)))
        {
            TCNT1 = 0;             // RESET COUNTER
            DDRB |= _BV(DDB6);     // OUTPUT OC1B (PB6, 10)
            TIFR1 |= _BV(OCF1B);   // CLEAR OCRB/OCRC FLAGS
            TIMSK1 |= _BV(OCIE1B); // ENABLE OCRB1 MATCH INTERRUPT
            TIFR3 |= _BV(TOV3);    // CLEAR OVERFLOW FLAG
            TIMSK3 |= _BV(TOIE3);  // ENABLE OVERFLOW INTERRUPT
        }
        OCR1B = ICR3 - ts;
        castlePwmRx = OCR1B; // KEEP PWM STATE FOR TELEMETRY PULSE LENGHT
        TCNT3 = 0;           // RESET COUNTER
#ifdef DEBUG_CASTLE_RX
        DEBUG_PRINT(castlePwmRx);
        DEBUG_PRINTLN();
#endif
    }
    TCCR3B ^= _BV(ICES3); // TOGGLE ICP3 EDGE
}

void EscCastle::TIMER3_OVF_handler() // NO RX INPUT
{
    DDRB &= ~_BV(DDB6);    // INPUT OC1B (PB6, 10)
    PORTB |= _BV(PB6);     // PB6 PULLUP
    TIMSK1 = 0;            // DISABLE INTERRUPTS
    TCCR3B |= _BV(ICES3);  // RISING EDGE
    TIMSK3 &= ~_BV(TOIE3); // DISABLE OVERFLOW INTERRUPT
#ifdef DEBUG_CASTLE_RX
    DEBUG_PRINT("X");
    DEBUG_PRINTLN();
#endif
}

void EscCastle::TIMER1_COMPB_handler() // START INPUT STATE
{
    DDRB &= ~_BV(DDB6);                 // INPUT OC1B (PB6)
    PORTB |= _BV(PB6);                  // PD2 PULLUP
    TIFR1 |= _BV(OCF1C) | _BV(ICF1);    // CLEAR ICP1 CAPTURE/OC1C FLAG
    TIMSK1 |= _BV(OCIE1C) | _BV(ICIE1); // ENABLE ICP1 CAPT/OC1C MATCH
}

void EscCastle::TIMER1_CAPT_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = ICR1 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_PRINT(castleTelemetry[castleCont]);
    DEBUG_PRINT(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
        castleUpdated = true;
    }
}

void EscCastle::TIMER1_COMPC_handler() // START OUTPUT STATE
{
    TIMSK1 &= ~_BV(ICIE1); // DISABLE ICP1 CAPT
    DDRB |= _BV(DDB6);     // OUTPUT OC1B (PB6,10)
    if (!castleTelemetryReceived)
    {
        castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
        castleCont = 0;
#ifdef DEBUG_CASTLE
        DEBUG_PRINTLN();
        DEBUG_PRINT("4 ");
        DEBUG_PRINT(millis());
        DEBUG_PRINT(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
void EscCastle::FTM1_IRQ_handler()
{
    if (FTM1_C0SC & FTM_CSC_CHF) // TIMER INPUT CAPTURE INTERRUPT RX
    {
        if (FTM1_C0V < 5000)
        {
            FTM0_C0SC |= FTM_CSC_CHIE;
            FTM0_C0V = castlePwmRx;
            castlePwmRx = FTM1_C0V;
#ifdef DEBUG_CASTLE_RX
            DEBUG_PRINT(castlePwmRx);
            DEBUG_PRINTLN();
#endif
        }
        FTM1_CNT = 0;
        FTM1_C0SC ^= FTM_CSC_ELSA;
        FTM1_C0SC ^= FTM_CSC_ELSB;
        FTM1_C0SC |= FTM_CSC_CHF; // CLEAR FLAG
    }
    else if (FTM1_SC & FTM_SC_TOF) // TIMER OVERFLOW INTERRUPT
    {
        //FTM0_C0SC &= ~0x3C;          // DISABLE CHANNEL
        FTM0_C0SC &= ~FTM_CSC_CHIE;
        FTM0_C4SC &= ~FTM_CSC_CHIE;
        FTM0_C2SC &= ~FTM_CSC_CHIE;
        PORTD_PCR0 = PORT_PCR_MUX(0); // PTD0 MUX 0 -> DISABLE
        FTM1_SC |= FTM_SC_TOF;        // CLEAR FLAG
#ifdef DEBUG_CASTLE_RX
        DEBUG_PRINT("X");
        DEBUG_PRINTLN();
#endif
    }
}

void EscCastle::FTM0_IRQ_handler()
{
    if (FTM0_C0SC & FTM_CSC_CHF) // CH0 INTERRUPT (DISABLE CH0 PWM OUT)
    {
        PORTD_PCR0 = PORT_PCR_MUX(0); // PTD0 MUX 0 -> DISABLE
        FTM0_C4SC |= FTM_CSC_CHF;     // CLEAR FLAG CH4
        FTM0_C4SC |= FTM_CSC_CHIE;    // ENABLE INTERRUPT CH4
        FTM0_C2SC |= FTM_CSC_CHF;     // CLEAR FLAG CH2
        FTM0_C2SC |= FTM_CSC_CHIE;    // ENABLE INTERRUPT CH2
        FTM0_C0SC |= FTM_CSC_CHF;     // CLEAR FLAG CH0
        if (!castleTelemetryReceived)
        {
            castleCont = 0;
            castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
#ifdef DEBUG_CASTLE
            DEBUG_PRINTLN();
            DEBUG_PRINT("3 ");
            DEBUG_PRINT(millis());
            DEBUG_PRINT(" ");
#endif
        }
        castleTelemetryReceived = false;
    }
    if (FTM0_C4SC & FTM_CSC_CHF) // CH4 INTERRUPT (CAPTURE TELEMETRY)
    {
        if (FTM0_C4V)
        {
            castleTelemetry[castleCont] = FTM0_C4V - castlePwmRx;

#ifdef DEBUG_CASTLE
            DEBUG_PRINT(castleTelemetry[castleCont]);
            DEBUG_PRINT(" ");
#endif
            if (castleCont < 11)
            {
                castleCont++;
                castleTelemetryReceived = true;
                castleUpdated = true;
            }
        }
        FTM0_C4SC |= FTM_CSC_CHF; // CLEAR FLAG CH4
    }
    if (FTM0_C2SC & FTM_CSC_CHF) // CH2 INTERRUPT (TOGGLE CH0 TO OUTPUT)
    {
        PORTD_PCR0 = PORT_PCR_MUX(4); // TPM0_CH0 MUX 4 -> PTD0 -> 2(PWM OUT)
        FTM0_C4SC &= ~FTM_CSC_CHIE;   // DISABLE INTERRUPT CH4
        FTM0_C2SC &= ~FTM_CSC_CHIE;   // DISABLE INTERRUPT CH2
        FTM0_C2SC |= FTM_CSC_CHF;     // CLEAR FLAG CH2
    }
}
#endif

void EscCastle::begin()
{
#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_COMPB_handlerP = TIMER1_COMPB_handler;
    INT0_handlerP = INT0_handler;
    //TIMER2_COMPA_handlerP = TIMER2_COMPA_handler;

    // TIMER 1, ESC: PWM OUTPUT, RX INPUT. ICP1 (PB0, PIN 8). OC1B (PB2 PIN 10) -> OUTPUT/INPUT PULL UP
    DDRB |= _BV(DDB2);                   // OUTPUT OC1B PB2 (PIN 10)
    PORTB |= _BV(PB0);                   // ICP1 PULLUP
    TCCR1A = _BV(WGM11) | _BV(WGM10);    // MODE 15
    TCCR1B = _BV(WGM13) | _BV(WGM12);    //
    TCCR1A |= _BV(COM1B1) | _BV(COM1B0); // TOGGLE OC1B ON OCR1B
    TCCR1B |= _BV(ICES1);                // RISING EDGE
    TCCR1B |= _BV(CS11);                 // SCALER 8
    TIMSK1 = _BV(ICIE1);                 // CAPTURE INTERRUPT
    OCR1A = 20 * MS_TO_COMP(8);          // 50Hz = 20ms

    // INT0. TELEMETRY INPUT (PD2, PIN2)
    EICRA = _BV(ISC01); // FALLING EDGE

    // TIMER 2. TOGGLE OC1B INPUT/OUTPUT
    TCCR2A = 0;                                 // NORMAL MODE
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // SCALER 1024
    OCR2A = 12 * MS_TO_COMP(1024);              // 12ms
#endif

#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_OVF_handlerP = TIMER1_OVF_handler;
    TIMER3_COMPA_handlerP = TIMER3_COMPA_handler;
    TIMER4_COMPB_handlerP = TIMER4_COMPB_handler;
    TIMER4_CAPT_handlerP = TIMER4_CAPT_handler;

    // TIMER1. RX INPUT. ICP1 (PB0, PIN 8)
    PORTB |= _BV(PB0);    // ICP1 PULLUP
    TCCR1A = 0;           //
    TCCR1B = 0;           // MODE 0 (NORMAL)
    TCCR1B |= _BV(ICES1); // RISING EDGE
    TCCR1B |= _BV(CS11);  // SCALER 8
    TIMSK1 = _BV(ICIE1);  // CAPTURE INTERRUPT

    // TIMER4. ESC: PWM OUTPUT, TELEMETRY INPUT. ICP4 (PE0, PIN 22). OC4B (PD2 PIN 2) -> OUTPUT/INPUT PULL UP
    TCCR4A = _BV(WGM41) | _BV(WGM40);    // MODE 15 (TOP OCR4A)
    TCCR4B = _BV(WGM43) | _BV(WGM42);    //
    TCCR4A |= _BV(COM4B1) | _BV(COM4B0); // TOGGLE OC4B ON OCR4B (INVERTING)
    TCCR4B &= ~_BV(ICES4);               // FALLING EDGE
    TCCR4B |= _BV(CS41);                 // SCALER 8
    TCCR4B |= _BV(ICNC4);                // NOISE CANCELLER
    OCR4A = 20 * MS_TO_COMP(8);          // 50Hz = 20ms

    // TIMER 3. TOGGLE OC4B INPUT/OUTPUT
    TCCR3A = 0;                     // NORMAL MODE
    TCCR3B = _BV(CS32) | _BV(CS30); // SCALER 1024
    OCR3A = 12 * MS_TO_COMP(1024);  // 12ms
#endif

#if defined(__AVR_ATmega2560__)
    TIMER4_CAPT_handlerP = TIMER4_CAPT_handler;
    TIMER4_OVF_handlerP = TIMER4_OVF_handler;
    TIMER5_COMPB_handlerP = TIMER5_COMPB_handler;
    TIMER5_COMPC_handlerP = TIMER5_COMPC_handler;
    TIMER5_CAPT_handlerP = TIMER5_CAPT_handler;

    // TIMER4. RX INPUT. ICP4 (PL0, PIN 49)
    PORTL |= _BV(PL0);    // ICP4 PULLUP
    TCCR4A = 0;           //
    TCCR4B = 0;           // MODE 0 (NORMAL)
    TCCR4B |= _BV(ICES4); // RISING EDGE
    TCCR4B |= _BV(CS41);  // SCALER 8
    TIMSK4 = _BV(ICIE4);  // CAPTURE INTERRUPT

    // TIMER5. ESC: PWM OUTPUT, TELEMETRY INPUT. ICP5 (PL1, PIN 48). OC5B (PL4 PIN 45) -> OUTPUT/INPUT PULL UP
    TCCR5A = _BV(WGM51) | _BV(WGM50);    // MODE 15 (TOP OCR5A)
    TCCR5B = _BV(WGM53) | _BV(WGM52);    //
    TCCR5A |= _BV(COM5B1) | _BV(COM5B0); // TOGGLE OC5B ON OCR5B (INVERTING)
    TCCR5B &= ~_BV(ICES5);               // FALLING EDGE
    TCCR5B |= _BV(CS51);                 // SCALER 8
    TCCR5B |= _BV(ICNC5);                // NOISE CANCELLER
    OCR5A = 20 * MS_TO_COMP(8);          // 50Hz = 20ms
    OCR5C = 12 * MS_TO_COMP(8);          // TOGGLE OC5B OUTPUT
#endif

#if defined(__AVR_ATmega32U4__)
    TIMER3_CAPT_handlerP = TIMER3_CAPT_handler;
    TIMER3_OVF_handlerP = TIMER3_OVF_handler;
    TIMER1_COMPC_handlerP = TIMER1_COMPC_handler;
    TIMER1_COMPB_handlerP = TIMER1_COMPB_handler;
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;

    // TIMER3. RX INPUT. ICP3 (PC7,13)
    PORTC |= _BV(PC7);    // ICP3 PULLUP
    TCCR3A = 0;           //
    TCCR3B = 0;           // MODE 0 (NORMAL)
    TCCR3B |= _BV(ICES3); // RISING EDGE
    TCCR3B |= _BV(CS31);  // SCALER 8
    TIMSK3 = _BV(ICIE3);  // CAPTURE INTERRUPT

    // TIMER1. ESC: PWM OUTPUT, TELEMETRY INPUT. ICP1 (PD4,4). OC1B (PB6,10) -> OUTPUT/INPUT PULL UP
    TCCR1A = _BV(WGM11) | _BV(WGM10);    // MODE 15 (TOP OCR1A)
    TCCR1B = _BV(WGM13) | _BV(WGM12);    //
    TCCR1A |= _BV(COM1B1) | _BV(COM1B0); // TOGGLE OC4B ON OCR4B (INVERTING)
    TCCR1B &= ~_BV(ICES1);               // FALLING EDGE
    TCCR1B |= _BV(CS11);                 // SCALER 8
    TCCR1B |= _BV(ICNC1);                // NOISE CANCELLER
    OCR1A = 20 * MS_TO_COMP(8);          // 50Hz = 20ms
    OCR1C = 12 * MS_TO_COMP(8);          // TOGGLE OC1C OUTPUT
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

    // FTM1 (2 CH): CAPTURE RX PULSE (PIN: CH0 -> PTB0 -> 16/A2)
    FTM1_IRQ_handlerP = FTM1_IRQ_handler;
    FTM1_SC = 0;
    delayMicroseconds(1);
    FTM1_CNT = 0;
    FTM1_MOD = 0xFFFF;
    SIM_SCGC6 |= SIM_SCGC6_FTM1;                           // ENABLE CLOCK
    FTM1_SC = FTM_SC_PS(5) | FTM_SC_CLKS(1) | FTM_SC_TOIE; // PRESCALER 32 | ENABLE COUNTER | ENABLE OVERFLOW INTERRUPT
    // CH0: INPUT CAPTURE
    FTM1_C0SC = 0;
    delayMicroseconds(1);
    FTM1_C0SC = FTM_CSC_ELSA | FTM_CSC_CHIE; // CAPTURE RISING
    // SET PIN
    PORTB_PCR0 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(3); // TPM1_CH0 MUX 3 -> PTB0 -> 16/A2 (CAPTURE INPUT RX)

    NVIC_ENABLE_IRQ(IRQ_FTM1);

    // FTM0 (6 CH): INVERTED PWM AT 20MHZ AND CAPTURE TELEMETRY (PINS: CH0 PWM OUTPUT (PTD0 -> 2), CH4 MUX 4 CAPTURE INPUT (PTD4 -> 6))
    FTM0_IRQ_handlerP = FTM0_IRQ_handler;
    FTM0_SC = 0;
    delayMicroseconds(1);
    FTM0_CNT = 0;
    SIM_SCGC6 |= SIM_SCGC6_FTM0;             // ENABLE CLOCK
    FTM0_SC = FTM_SC_PS(5) | FTM_SC_CLKS(1); // PRESCALER 32 | ENABLE COUNTER
    FTM0_MOD = 20 * MS_TO_COMP(32);          // 20ms (100HZ)
    // CH0: PWM OUTPUT (ESC CONTROL)
    FTM0_C0SC = 0;
    delayMicroseconds(1);
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSA; // OUTPUT | LOW PULSES
    // CH4: INPUT TELEMETRY
    FTM0_C4SC = 0;
    delayMicroseconds(1);
    FTM0_C4SC = FTM_CSC_ELSB; // CAPTURE FALLING
    // CH2: TOGGLE CH0 TO OUTPUT
    FTM0_C2SC = 0;
    delayMicroseconds(1);
    FTM0_C2SC = FTM_CSC_MSA;        // SOFTWARE COMPARE
    FTM0_C2V = 12 * MS_TO_COMP(32); // 12ms TOGGLE CH0 TO OUTPUT
    // SET PINS
    PORTD_PCR0 = PORT_PCR_MUX(4);               // TPM0_CH0 MUX 4 -> PTD0 -> 2 (PWM OUT)
    PORTD_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_PE; // TPM0_CH4 MUX 4 -> PTD4 -> 6 (CAPTURE), PULLUP

    NVIC_ENABLE_IRQ(IRQ_FTM0);
#endif
}

float EscCastle::getValue(uint8_t index)
{
    float value = ((float)castleTelemetry[index] / castleCompsPerMilli - 0.5) * scaler[index];
    if (value < 0)
        value = 0;
    return value;
}

void EscCastle::update()
{
    if (castleUpdated)
    {
        voltage_ = getValue(1);
        rippleVoltage_ = getValue(2);
        current_ = getValue(3);
        thr_ = getValue(4);
        output_ = getValue(5);
        rpm_ = getValue(6) * RPM_MULTIPLIER;
        consumption_ += calcConsumption(current_);
        becVoltage_ = getValue(7);
        becCurrent_ = getValue(8);
        if (castleTelemetry[9] > castleTelemetry[10])
        {
            temperature_ = getValue(9);
        }
        else
        {
            float ntc_value = getValue(10);
            temperature_ = 1 / (log(ntc_value * CASTLE_R2 / (255.0F - ntc_value) / CASTLE_R0) / CASTLE_B + 1 / 298.0F) - 273.0F;
        }
        if (temperature_ < 0)
            temperature_ = 0;
        if (cellCount_ == 255)
            if (millis() > 10000 && voltage_ > 1)
                cellCount_ = setCellCount(voltage_);
        cellVoltage_ = voltage_ / cellCount_;
        castleUpdated = false;
    }
}

float *EscCastle::voltageP()
{
    return &voltage_;
}

float *EscCastle::rippleVoltageP()
{
    return &rippleVoltage_;
}

float *EscCastle::currentP()
{
    return &current_;
}

float *EscCastle::thrP()
{
    return &thr_;
}

float *EscCastle::outputP()
{
    return &output_;
}

float *EscCastle::rpmP()
{
    return &rpm_;
}

float *EscCastle::consumptionP()
{
    return &consumption_;
}

float *EscCastle::becVoltageP()
{
    return &becVoltage_;
}

float *EscCastle::becCurrentP()
{
    return &becCurrent_;
}

float *EscCastle::temperatureP()
{
    return &temperature_;
}

float *EscCastle::cellVoltageP()
{
    return &cellVoltage_;
}
