#include "escCastle.h"

volatile bool EscCastle::castleTelemetryReceived = false;
#ifdef SIM_SENSORS
volatile uint16_t EscCastle::castleTelemetry[12] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 500};
#else
volatile uint16_t EscCastle::castleTelemetry[12] = {0};
#endif
volatile uint16_t EscCastle::castleCompsPerMilli = 1 * CASTLE_MS_TO_COMP(8);
volatile uint8_t EscCastle::castleCont = 0;
volatile uint8_t EscCastle::castleRxLastReceived = 0;

EscCastle::EscCastle(uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp) {}

#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)
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
    TIFR2 |= _BV(OCF2A);  // CLEAR TIMER2 COMPA FLAG
    TIMSK2 = _BV(OCIE2A); // ENABLE TIMER2 COMPA
    TCNT2 = 0;            // RESET TIMER2
}

void EscCastle::INT0_handler() // READ TELEMETRY
{
    castleTelemetry[castleCont] = TCNT1 - OCR1B;
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
    if (castleRxLastReceived > RX_MAX_CYCLES)
    {
        OCR1B = 0;
        TIMSK1 &= ~_BV(OCIE1B); // DISABLE INPUT STATE
    }
    else
    {
        castleRxLastReceived++;
    }
    EIMSK = 0;         // DISABLE INT0 (PD2, PIN2)
    TIMSK2 = 0;        // DISABLE TIMER2 INTS
    DDRB |= _BV(DDB2); // PWM OUT PIN 10 OUTPUT
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
            TIFR4 |= _BV(OCF4B);   // CLEAR OCRB CAPTURE FLAG
            TIMSK4 |= _BV(OCIE4B); // ENABLE OCR MATCH INTERRUPT
        }
        OCR4B = ICR1;
        castleRxLastReceived = 0;
    }
    TCCR1B ^= _BV(ICES1); // TOGGLE ICP1 EDGE
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
    castleTelemetry[castleCont] = TCNT4 - OCR4B;
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
    if (castleRxLastReceived > RX_MAX_CYCLES)
    {
        OCR4B = 0;              // DISABLE PWM
        TIMSK4 &= ~_BV(OCIE4B); // DISABLE OCR MATCH INTERRUPT
    }
    else
    {
        castleRxLastReceived++;
    }
    TIMSK4 &= ~_BV(ICIE4);  // DISABLE ICP4 CAPT
    DDRD |= _BV(DDD2);      // OUTPUT OC4B (PD2, 2)
    TIMSK2 &= ~_BV(OCIE2A); // DISABLE TIMER2 OCRA INTERRUPT
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
    TCCR1B = 0;           // MODE 0 (NORMAL)
    TCCR1B |= _BV(ICES1); // RISING EDGE
    TCCR1B |= _BV(CS11);  // SCALER 8
    TIMSK1 = _BV(ICIE1);  // CAPTURE INTERRUPT

    // TIMER4. ESC: PWM OUTPUT, TELEMETRY INPUT. ICP4 (PE0, PIN 22). OC4B (PD2 PIN 2) -> OUTPUT/INPUT PULL UP
    DDRD |= _BV(DDD2);                   // OUTPUT OC4B PD2 (PIN 2)
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
#ifdef DEBUG_ESC
    DEBUG_SERIAL.print("Value [");
    DEBUG_SERIAL.print(index);
    DEBUG_SERIAL.print("]: ");
    DEBUG_SERIAL.println(value);
#endif
    if (value < 0)
        return 0;
    return value;
}