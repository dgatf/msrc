#include "softserial.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)

ISR(PCINTx_vect)
{
    softSerial.PCINT_handler();
}

ISR(TIMER2_COMPB_vect)
{
    softSerial.TIMER_COMP_handler();
}

SoftSerial::SoftSerial() {}

void SoftSerial::TIMER_COMP_handler()
{
    TIMSK2 &= ~_BV(OCIE2B);
    timedout = true;
}

void SoftSerial::PCINT_handler()
{
    if (inverted_ ? bit_is_set(PINx, PINxn) : !bit_is_set(PINx, PINxn))
    {
        PCMSKx &= ~_BV(PCINTxn);
        uint8_t incomingByte = 0;

        // start bit
        _delay_loop_2(rx_delay_centering);

        // data
        for (uint8_t i = 0; i < 8; i++)
        {

            _delay_loop_2(rx_delay);
            if (bit_is_set(PINx, PINxn))
                incomingByte |= _BV(i);

            /*
      _delay_loop_2(rx_delay);
      incomingByte >>= 1;
      if ( bit_is_set(PINx, PINxn) )
        incomingByte |= 0x80;
      */
        }
        if (inverted_)
            incomingByte = ~incomingByte;

        // stop bit
        //_delay_loop_2(rx_delay_stop);
        if (timedout)
            reset();
        writeRx(incomingByte);
        timedout = false;
        PCIFR = B111;
        PCMSKx |= _BV(PCINTxn);
        OCR2B = TCNT2 + timeout_;
        TIFR2 |= _BV(OCF2B);
        TIMSK2 |= _BV(OCIE2B);
    }
}

void SoftSerial::initWrite()
{
    //DDRB |= _BV(DDB4);

    uint8_t outgoingByte = readTx();
    uint8_t oldSREG = SREG;
    bool inv = inverted_;
    uint16_t delay = tx_delay;

    if (inv)
        outgoingByte = ~outgoingByte;

    cli();

    // start bit
    if (inv)
    {
        setPinHigh;
    }
    else
    {
        setPinLow;
    }
    _delay_loop_2(delay);

    // data
    for (uint8_t i = 0; i < 8; i++)
    {
        if (outgoingByte & 1)
        {
            setPinHigh;
        }
        else
        {
            setPinLow;
        }
        _delay_loop_2(delay);
        outgoingByte >>= 1;
    }

    // stop bit
    if (inv)
    {
        setPinLow;
    }
    else
    {
        setPinHigh;
    }
    SREG = oldSREG;
    _delay_loop_2(tx_delay);

    //DDRB &= ~_BV(DDB4);
}

uint8_t SoftSerial::availableTimeout()
{
    if (timedout)
        return available();
    return 0;
}

void SoftSerial::setTimeout(uint8_t timeout)
{
    timeout_ = timeout * MS_TO_COMP(1024);
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

    inverted_ = format & 0x40;
    if (inverted_)
        setPinLow;
    else
        setPinHigh;

    // 1 bit delay in 4 clock cycles
    uint16_t delay = (F_CPU / baud) / 4;
    // substract overheads
    tx_delay = subs(delay, 15 / 4); // for teensylc//atmega2560, 100000: 23
    rx_delay = subs(delay, 23 / 4); // for teensylc/atmega2560, 100000: 16
    rx_delay_centering = subs(delay / 2, (4 + 4 + 75 + 17 - 16) / 4);
    rx_delay_stop = subs(delay * 3 / 4, (37 + 11) / 4);

    // Set TMR2 to measure ms (max 16Mhz 16ms, 8Mhz 32ms)
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // SCALER 1024
    TCCR2A = 0;
}

uint16_t SoftSerial::subs(uint16_t val1, uint16_t val2)
{
    if (val1 > val2)
        return val1 - val2;
    return 1;
}

SoftSerial softSerial;

#endif