#include "escCastle.h"

volatile bool castleReceived = false;
#ifdef SIM_SENSORS
volatile uint16_t castleTelemetry[12] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
#else
volatile uint16_t castleTelemetry[12] = {0};
#endif
volatile int castleCont = 0;
volatile uint16_t compToMilli = 0;

ISR(INT0_vect)
{
    if (PIND & _BV(PD2))
    {                       // PD2 high
        DDRD |= _BV(DDD3);  // PD3 output
        PORTD &= ~_BV(PD3); // PD3 low
    }
    else
    {                        // PD2 low
        DDRD &= ~_BV(DDD3);  // PD3 input
        PORTD |= _BV(PD3);   // PD3 pullup
        TCNT1 = 0;           // reset TIMER1
        EIFR &= ~_BV(INTF1); // remove INT1 flags
        EIMSK |= _BV(INT1);  // enable INT1 (PD3)
        if (!castleReceived)
        {
            castleCont = 0;
            compToMilli = castleTelemetry[0] / 2 + (castleTelemetry[9] < castleTelemetry[10]) ? castleTelemetry[9] : castleTelemetry[10];
        }
        castleReceived = false;
    }
}

ISR(INT1_vect)
{
    if (~PIND & _BV(PD2)) // remove?
    {
        castleTelemetry[castleCont] = TCNT1;
        castleReceived = true;
        castleCont++;
        Serial.println(castleTelemetry[castleCont]);
    }
}

EscCastleInterface::EscCastleInterface(uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp) {}

void EscCastleInterface::begin()
{
    // rx pin 2 (PD2, INT 0): input, esc pin 3 (PD3, INT 1): input/output
    EICRA = _BV(ISC00);  // INT0 rising/falling
    EICRA |= _BV(ISC11); // INT1 falling
    EIMSK = _BV(INT0);   // enable INT0 (PIN 2)
    TCCR1A = 0;          // normal mode
    TCCR1B = _BV(CS11);  // scaler 8
}

float EscCastleInterface::read(uint8_t index)
{
    float value;
    switch (index)
    {
    case 11:
        if (cellCount_ == 255 && millis() > 10000)
        {
            cellCount_ = setCellCount(((float)castleTelemetry[CASTLE_VOLTAGE] / compToMilli - 0.5) * scaler[CASTLE_VOLTAGE]);
        }
        value = ((float)castleTelemetry[index] / compToMilli - 0.5) * scaler[index] / cellCount_;
    case 0 ... 8:
        value = ((float)castleTelemetry[index] / compToMilli - 0.5) * scaler[index];
        break;
    case 9:
        if (castleTelemetry[9] > castleTelemetry[10])
        {
            value = ((float)castleTelemetry[index] / compToMilli - 0.5) * scaler[index];
        }
        else
        {
            value = 0;
        }
        break;
    case 10:
        if (castleTelemetry[10] > castleTelemetry[9])
        {
            float ntc_value = ((float)castleTelemetry[index] / compToMilli - 0.5) * scaler[index + 1];
            value = 1 / (log(ntc_value * R2 / (255 - ntc_value) / R0) / B + 1 / 298) - 273;
        }
        else
        {
            value = 0;
        }
        break;
    default:
        value = 0;
        break;
    }
#ifdef DEBUG_ESC
    Serial.print("Value: ");
    Serial.println(value);
#endif
    return value;
}