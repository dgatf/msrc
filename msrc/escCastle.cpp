#include "escCastle.h"

volatile bool EscCastle::castleTelemetryReceived = false;
#ifdef SIM_SENSORS
volatile uint16_t EscCastle::castleTelemetry[12] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 500};
#else
volatile uint16_t EscCastle::castleTelemetry[12] = {0};
#endif
volatile uint16_t EscCastle::castleCompsPerMilli = 1 * CASTLE_MS_TO_COMP(8);
volatile uint8_t EscCastle::castleCont = 0;
volatile uint16_t EscCastle::castlePwmRx = 0;

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
    }
    TCCR1B ^= _BV(ICES1); // TOGGLE ICP1 DIRECTION
}

void EscCastle::TIMER1_COMPB_handler() // START INPUT STATE
{
    DDRB &= ~_BV(DDB2);   // PWM OUT (PB2, PIN10) INPUT
    PORTB |= _BV(PB2);    // PB2 PULLUP
    EIFR |= _BV(INTF0);   // CLEAR INT0 FLAG
    EIMSK = _BV(INT0);    // ENABLE INT0 (PD2, PIN2)
    TCNT2 = 0;            // RESET TIMER2 COUNTER
    TIFR2 |= _BV(OCF2A);  // CLEAR TIMER2 OCRA CAPTURE FLAG
    TIMSK2 = _BV(OCIE2A); // ENABLE TIMER2 OCRA INTERRUPT
}

void EscCastle::INT0_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = TCNT1 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_SERIAL.print(castleTelemetry[castleCont]);
    DEBUG_SERIAL.print(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
    }
}

void EscCastle::TIMER2_COMPA_handler() // START OUTPUT STATE
{
    EIMSK = 0;         // DISABLE INT0 (PD2, PIN2)
    TIMSK2 = 0;        // DISABLE TIMER2 INTS
    DDRB |= _BV(DDB2); // PWM OUT PIN 10 OUTPUT
    if (castleRxLastReceived > RX_MAX_CYCLES)
    {
        OCR1B = 0;
        TIMSK1 &= ~_BV(OCIE1B); // DISABLE INPUT STATE
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
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.print(millis());
        DEBUG_SERIAL.print(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
void EscCastle::TIMER1_CAPT_handler() // RX INPUT
{
    if (TCCR1B & _BV(ICES1)) // RX RISING (PULSE START)
    {
        TCNT1 = 0; // RESET COUNTER
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
        OCR4B = ICR1;
        castlePwmRx = OCR4B;  // KEEP PWM STATE FOR TELEMETRY PULSE LENGHT
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
    TCNT2 = 0;            // RESET TIMER2 COUNTER
    TIFR2 |= _BV(OCF2A);  // CLEAR TIMER2 OCRA CAPTURE FLAG
    TIMSK2 = _BV(OCIE2A); // ENABLE TIMER2 OCRA INTERRUPT
}

void EscCastle::TIMER4_CAPT_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = TCNT4 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_SERIAL.print(castleTelemetry[castleCont]);
    DEBUG_SERIAL.print(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
    }
}

void EscCastle::TIMER2_COMPA_handler() // START OUTPUT STATE
{
    TIMSK4 &= ~_BV(ICIE4);  // DISABLE ICP4 CAPT
    DDRD |= _BV(DDD2);      // OUTPUT OC4B (PD2, 2)
    TIMSK2 &= ~_BV(OCIE2A); // DISABLE TIMER2 OCRA INTERRUPT
    if (!castleTelemetryReceived)
    {
        castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
        castleCont = 0;
#ifdef DEBUG_CASTLE
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.print(millis());
        DEBUG_SERIAL.print(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

#if defined(__AVR_ATmega2560__)
void EscCastle::TIMER4_CAPT_handler() // RX INPUT
{
    if (TCCR4B & _BV(ICES4)) // RX RISING (PULSE START)
    {
        TCNT4 = 0; // RESET COUNTER
    }
    else // RX FALLING (PULSE END)
    {
        if (!(TIMSK5 & _BV(OCIE5B)))
        {
            TCNT5 = 0;             // RESET COUNTER
            DDRL &= ~_BV(DDL4);    // INPUT OC5B (PL4, 45)
            TIFR5 |= _BV(OCF5B);   // CLEAR OCRB CAPTURE FLAG
            TIMSK5 |= _BV(OCIE5B); // ENABLE OCRB MATCH INTERRUPT
            TIFR4 |= _BV(TOV4);    // CLEAR OVERFLOW FLAG
            TIMSK4 |= _BV(TOIE4);  // ENABLE OVERFLOW INTERRUPT
        }
        OCR5B = ICR4;
        castlePwmRx = OCR5B;  // KEEP PWM STATE FOR TELEMETRY PULSE LENGHT
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
}

void EscCastle::TIMER5_COMPB_handler() // START INPUT STATE
{
    DDRL &= ~_BV(DDL4);   // INPUT OC5B (PL4, 45)
    PORTL |= _BV(PL4);    // PL4 PULLUP
    TIFR5 |= _BV(ICF5);   // CLEAR ICP5 CAPTURE FLAG
    TIMSK5 |= _BV(ICIE5); // ENABLE ICP5 CAPT
    TCNT2 = 0;            // RESET TIMER2 COUNTER
    TIFR2 |= _BV(OCF2A);  // CLEAR TIMER2 OCRA CAPTURE FLAG
    TIMSK2 = _BV(OCIE2A); // ENABLE TIMER2 OCRA INTERRUPT
}

void EscCastle::TIMER5_CAPT_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = TCNT5 - castlePwmRx;
#ifdef DEBUG_CASTLE
    DEBUG_SERIAL.print(castleTelemetry[castleCont]);
    DEBUG_SERIAL.print(" ");
#endif
    if (castleCont < 11)
    {
        castleCont++;
        castleTelemetryReceived = true;
    }
}

void EscCastle::TIMER2_COMPA_handler() // START OUTPUT STATE
{
    TIMSK5 &= ~_BV(ICIE5);  // DISABLE ICP5 CAPT
    DDRL |= _BV(DDL4);      // OUTPUT OC5B (PL4, 45)
    TIMSK2 &= ~_BV(OCIE2A); // DISABLE TIMER2 OCRA INTERRUPT
    if (!castleTelemetryReceived)
    {
        castleCompsPerMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10] ? castleTelemetry[9] : castleTelemetry[10]);
        castleCont = 0;
#ifdef DEBUG_CASTLE
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.print(millis());
        DEBUG_SERIAL.print(" ");
#endif
    }
    castleTelemetryReceived = false;
}
#endif

void EscCastle::begin()
{
#ifdef DEBUG_CASTLE
    ESC_SERIAL.begin(115200);
#endif

#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER1_COMPB_handlerP = TIMER1_COMPB_handler;
    INT0_handlerP = INT0_handler;
    TIMER2_COMPA_handlerP = TIMER2_COMPA_handler;

    // TIMER 1, ESC: PWM OUTPUT, RX INPUT. ICP1 (PB0, PIN 8). OC1B (PB2 PIN 10) -> OUTPUT/INPUT PULL UP
    DDRB |= _BV(DDB2);                  // OUTPUT OC1B PB2 (PIN 10)
    TCCR1A = _BV(WGM11) | _BV(WGM10);   // MODE 15
    TCCR1B = _BV(WGM13) | _BV(WGM12);   //
    TCCR1A = _BV(COM1B1) | _BV(COM1B0); // TOGGLE OC1B ON OCR1B
    TCCR1B |= _BV(ICES1);               // RISING EDGE
    TCCR1B |= _BV(CS11);                // SCALER 8
    TIMSK1 = _BV(ICIE1);                // CAPTURE INTERRUPT
    OCR1A = 20 * CASTLE_MS_TO_COMP(8);  // 50Hz = 20ms

    // INT0. TELEMETRY INPUT (PD2, PIN2)
    EICRA = _BV(ISC01); // FALLING EDGE

    // TIMER 2. TOGGLE OC1B INPUT/OUTPUT
    TCCR2A = 0;                                 // NORMAL MODE
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // SCALER 1024
    OCR2A = 12 * CASTLE_MS_TO_COMP(1024);       // 12ms
#endif

#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
    TIMER1_CAPT_handlerP = TIMER1_CAPT_handler;
    TIMER2_COMPA_handlerP = TIMER2_COMPA_handler;
    TIMER4_COMPB_handlerP = TIMER4_COMPB_handler;
    TIMER4_CAPT_handlerP = TIMER4_CAPT_handler;

    // TIMER1. RX INPUT. ICP1 (PB0, PIN 8)
    PORTB |= _BV(PB0);     // ICP1 PULLUP
    TCCR1A = 0;
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
    OCR4A = 20 * CASTLE_MS_TO_COMP(8);   // 50Hz = 20ms

    // TIMER 2. TOGGLE OC4B INPUT/OUTPUT
    TCCR2A = 0;                                 // NORMAL MODE
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // SCALER 1024
    OCR2A = 12 * CASTLE_MS_TO_COMP(1024);       // 12ms
#endif

#if defined(__AVR_ATmega2560__)
    TIMER4_CAPT_handlerP = TIMER4_CAPT_handler;
    TIMER2_COMPA_handlerP = TIMER2_COMPA_handler;
    TIMER5_COMPB_handlerP = TIMER5_COMPB_handler;
    TIMER5_CAPT_handlerP = TIMER5_CAPT_handler;

    // TIMER4. RX INPUT. ICP4 (PL0, PIN 49)
    PORTL |= _BV(PL0);    // ICP3 PULLUP
    TCCR4A = 0;           //
    TCCR4B = 0;           // MODE 0 (NORMAL)
    TCCR4B |= _BV(ICES4); // RISING EDGE
    TCCR4B |= _BV(CS41);  // SCALER 8
    TIMSK4 = _BV(ICIE4);  // CAPTURE INTERRUPT

    // TIMER5. ESC: PWM OUTPUT, TELEMETRY INPUT. ICP5 (Pl1, PIN 48). OC5B (Pl4 PIN 45) -> OUTPUT/INPUT PULL UP
    TCCR5A = _BV(WGM51) | _BV(WGM50);    // MODE 15 (TOP OCR5A)
    TCCR5B = _BV(WGM53) | _BV(WGM52);    //
    TCCR5A |= _BV(COM5B1) | _BV(COM5B0); // TOGGLE OC5B ON OCR5B (INVERTING)
    TCCR5B &= ~_BV(ICES5);               // FALLING EDGE
    TCCR5B |= _BV(CS51);                 // SCALER 8
    OCR5A = 20 * CASTLE_MS_TO_COMP(8);   // 50Hz = 20ms

    // TIMER 2. TOGGLE OC5B INPUT/OUTPUT
    TCCR2A = 0;                                 // NORMAL MODE
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // SCALER 1024
    OCR2A = 12 * CASTLE_MS_TO_COMP(1024);       // 12ms
#endif
}

float EscCastle::read(uint8_t index)
{
    float value;
    if (cellCount_ == 255)
    {
        if (millis() > 10000)
        {
            cellCount_ = setCellCount(((float)castleTelemetry[CASTLE_VOLTAGE] / castleCompsPerMilli - 0.5) * scaler[CASTLE_VOLTAGE]);
        }
    }
    switch (index)
    {
    case 0 ... 8:
        value = ((float)castleTelemetry[index] / castleCompsPerMilli - 0.5) * scaler[index];
        break;
    case 9:
        if (castleTelemetry[9] > castleTelemetry[10])
        {
            value = ((float)castleTelemetry[index] / castleCompsPerMilli - 0.5) * scaler[index];
        }
        else
        {
            value = 0;
        }
        break;
    case 10:
        if (castleTelemetry[10] > castleTelemetry[9])
        {
            float ntc_value = ((float)castleTelemetry[index] / castleCompsPerMilli - 0.5) * scaler[index];
            value = 1 / (log(ntc_value * CASTLE_R2 / (255.0F - ntc_value) / CASTLE_R0) / CASTLE_B + 1 / 298.0F) - 273.0F;
        }
        else
        {
            value = 0;
        }
        break;
    case 11:
        value = (((float)castleTelemetry[1] / castleCompsPerMilli - 0.5) * scaler[1]) / cellCount_;
        break;
    default:
        value = 0;
        break;
    }
#ifdef DEBUG_CASTLE
    DEBUG_SERIAL.print("Value [");
    DEBUG_SERIAL.print(index);
    DEBUG_SERIAL.print("]: ");
    DEBUG_SERIAL.println(value);
#endif
    if (value < 0)
        return 0;
    return value;
}