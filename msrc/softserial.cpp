#include "softserial.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)

ISR(PCINTx_vect)
{
    softSerial.PCINT2_handler();
}

ISR(TIMER0_COMPA_vect)
{
    softSerial.TIMER0_COMPA_handler();
}

SoftSerial::SoftSerial() {}

void SoftSerial::PCINT2_handler()
{
    if (status == SOFTSERIAL_IDLE)
    {
        if ((!bit_is_set(PINx, PINxn) && !inverted_) || (bit_is_set(PINx, PINxn) && inverted_))
        {
            initVal = TCNT0 + initDeltaRx;
            OCR0A = initVal + delta[0];
            TIFR0 |= _BV(OCF0A);
            TIMSK0 |= _BV(OCIE0A);
            PCMSKx &= ~_BV(PCINTxn);
            bit = 1;
            incomingByte = 0;
            status = SOFTSERIAL_RECEIVING;
            if ((uint16_t)micros() - ts > timeout_ * 1000)
                reset();
        }
    }
}

void SoftSerial::TIMER0_COMPA_handler() // Rx, Tx
{
    if (status == SOFTSERIAL_RECEIVING)
    {
        if (bit <= 8) // data
        {
            if ((bit_is_set(PINx, PINxn) && !inverted_) || (!bit_is_set(PINx, PINxn) && inverted_))
                incomingByte |= _BV(bit - 1);
            OCR0A = initVal + delta[bit];
            bit++;
        }
        else // end of transmission
        {
            PCMSKx |= _BV(PCINTxn);
            TIMSK0 &= ~_BV(OCIE0A);
            writeRx(incomingByte);
            status = SOFTSERIAL_IDLE;
            ts = micros();
        }
    }
    else
    {
        if (bit < 9) // data
        {
            if (bit_is_set(outgoingByte, bit - 1))
            {
                setPinLogic1;
            }
            else
                setPinLogic0;
        }
        else if (bit == 9) // stop bit
        {
            setPinLogic1;
        }
        else // end of transmission
        {
            status = SOFTSERIAL_IDLE;
            if (availableTx())
            {
                initWrite();
                return;
            }
            else
            {
                PCMSKx |= _BV(PCINTxn);
                TIFR0 |= _BV(OCF0A);
                TIMSK0 &= ~_BV(OCIE0A);
            }
            return;
        }
        OCR0A = initVal + delta[bit];
        bit++;
    }
}

void SoftSerial::initWrite()
{
    if (status == SOFTSERIAL_IDLE)
    {
        PCMSKx &= ~_BV(PCINTxn);
        status = SOFTSERIAL_SENDING;
        bit = 1;
        outgoingByte = readTx();
        initVal = TCNT0 - initDeltaTx;
        OCR0A = initVal + delta[0];
        setPinLogic0;
        TIFR0 |= _BV(OCF0A);
        TIMSK0 |= _BV(OCIE0A);
    }
}

void SoftSerial::begin(uint32_t baud, uint8_t format)
{
    // FORMAT: 8N1

    // RX: 328P/PB: PIN 7   (PD7,PCINT23)
    //     2560:    PIN A15 (PK7,PCINT23)
    //     32U4:    PIN B3  (PB3,PCINT3)

    // TX: 328P/PB: PIN 12  (PB4)
    //     2560:    PIN D10 (PB4)
    //     32U4:    PIN B4  (PB4)

    PORTx |= _BV(PORTxn);   // PULLUP
    PCICR |= _BV(PCIEx);    // ENABLE PCINT
    PCMSKx |= _BV(PCINTxn); // PCINT MASK

    DDRB |= _BV(DDB4);

    TCCR0A = 0;

    inverted_ = format & 0x40;
    setPinLogic1;

    float comp = 1000.0 / baud * MS_TO_COMP(64);
    for (uint8_t i = 0; i < 12; i++)
        delta[i] = round(comp * (i + 1));
    initDeltaRx = round(0.0 * comp);
    initDeltaTx = round(0.3 * comp);
}

SoftSerial softSerial;

#endif