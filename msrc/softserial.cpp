#include "softserial.h"

#if defined(__AVR_ATmega328P__) ||  defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)

ISR(PCINTx_vect)
{
    softSerial.PCINT_handler();
}

SoftSerial::SoftSerial() {}

uint8_t SoftSerial::availableTimeout()
{
    uint8_t availableCopy = available();
    if (timedout)
        return availableCopy;
    return 0;
}

uint16_t SoftSerial::subs(uint16_t val1, uint16_t val2)
{
    if (val1 > val2)
        return val1 - val2;
    return 1;
}

SoftSerial softSerial;

#endif

#if defined(__AVR_ATmega32U4__)

ISR(TIMER3_COMPA_vect)
{
    softSerial.TIMER_COMP_handler();
}

void SoftSerial::TIMER_COMP_handler()
{
    TIMSK3 &= ~_BV(OCIE3A);
    timedout = true;
    ts = micros();
}

inline void SoftSerial::delay_loop(uint16_t delay)
{
    asm volatile (
        "1: sbiw %0,1" "\n\t"
        "brne 1b"
        : "=w" (delay)
        : "0" (delay)
    );

    /*uint8_t tmp = 0;
    asm volatile("sbiw    %0, 0x01 \n\t"
                 "ldi %1, 0xFF \n\t"
                 "cpi %A0, 0xFF \n\t"
                 "cpc %B0, %1 \n\t"
                 "brne .-10 \n\t"
                 : "+r"(delay), "+a"(tmp)
                 : "0"(delay)
    );*/

}

void SoftSerial::PCINT_handler()
{
    if (inverted_ ? bit_is_set(PINx, PINxn) : !bit_is_set(PINx, PINxn))
    {
        PCMSKx &= ~_BV(PCINTxn);
        uint8_t incomingByte = 0;

        // start bit
        delay_loop(rx_delay_centering);

        // data
        for (uint8_t i = 8; i > 0; --i)
        {
            delay_loop(rx_delay);
            incomingByte >>= 1;
            if (bit_is_set(PINx, PINxn))
                incomingByte |= 0x80;
        }

        if (inverted_)
            incomingByte = ~incomingByte;

        // stop bits
        delay_loop(rx_delay_stop);

        if (timedout)
            reset();
        writeRx(incomingByte);
        timedout = false;
        PCIFR = B111;
        PCMSKx |= _BV(PCINTxn);
        OCR3A = TCNT3 + timeout_;
        TIFR3 |= _BV(OCF3A);
        TIMSK3 |= _BV(OCIE3A);
    }
}

void SoftSerial::initWrite()
{
    DDRB |= _BV(DDB4);

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
    delay_loop(delay);

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
        delay_loop(delay);
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

    if (half_duplex_)
        DDRB &= ~_BV(DDB4);

    SREG = oldSREG;
    delay_loop(delay);
}

void SoftSerial::setTimeout(uint16_t timeout)
{
    timeout_ = timeout * US_TO_COMP(8);
}

uint16_t SoftSerial::timestamp()
{
    return (uint16_t)(micros() - ts) + timeout_ / US_TO_COMP(8);
}

void SoftSerial::begin(uint32_t baud, uint8_t format)
{
    // FORMAT: 8N1

    // RX: 328P/PB: PIN 7   (PD7,PCINT23)
    //     2560:    PIN A15 (PK7,PCINT23)
    //     32U4:    PIN B3/14  (PB3,PCINT3)

    // TX: 328P/PB: PIN 12  (PB4)
    //     2560:    PIN D10 (PB4)
    //     32U4:    PIN B4/8  (PB4)

    PCICR |= _BV(PCIEx);    // ENABLE PCINT
    PCMSKx |= _BV(PCINTxn); // PCINT MASK

    inverted_ = format & 0x40;
    half_duplex_ = format & 0x80;

    if (!half_duplex_)
    {
        DDRB |= _BV(DDB4);
        if (inverted_)
            setPinLow;
        else
            setPinHigh;
    }
    else if (!inverted_)
        PORTx |= _BV(PORTxn); // RX PULLUP for half duplex for non inverted signal. For inverted signal, an external pull down resistor maybe needed. Not needed if the other side has one

    // 1 bit delay in 4 clock cycles
    uint16_t delay = (F_CPU / baud) / 4;
    // substract overheads
    tx_delay = subs(delay, 17 / 4); //15
    rx_delay = subs(delay, 15 / 4); //23
    rx_delay_centering = subs(delay / 2, (4 + 4 + 75 + 17 - 15) / 4);
    rx_delay_stop = subs(delay * 3 / 4, (37 + 11) / 4);

    TCCR3A = 0;
    TCCR3B = _BV(CS31); // SCALER 8
}

#endif

#if defined(__AVR_ATmega328P__) ||  defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

ISR(TIMER2_COMPB_vect)
{
    softSerial.TIMER_COMP_handler();
}

void SoftSerial::TIMER_COMP_handler()
{
    TIMSK2 &= ~_BV(OCIE2B);
    timedout = true;
    ts = micros();
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
        /*
        for (uint8_t i = 0; i < 8; i++)
        {
            _delay_loop_2(rx_delay);
            if (bit_is_set(PINx, PINxn))
                incomingByte |= _BV(i);
        }
        */
        for (uint8_t i = 8; i > 0; --i)
        {
            _delay_loop_2(rx_delay);
            incomingByte >>= 1;
            if (bit_is_set(PINx, PINxn))
                incomingByte |= 0x80;
        }

        if (inverted_)
            incomingByte = ~incomingByte;

        // stop bits
        _delay_loop_2(rx_delay_stop);

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
    DDRB |= _BV(DDB4);

    uint8_t outgoingByte = readTx();
    uint8_t oldSREG = SREG;
    bool inv = inverted_;
    uint16_t delay = tx_delay;
    uint16_t delay_stop = tx_delay_stop;
    uint8_t parity = parity_;
    uint8_t parity_value = 0;

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
        parity_value ^= outgoingByte & 1;
        _delay_loop_2(delay);
        outgoingByte >>= 1;
    }

    // parity
    if (parity)
    {
        parity_value ^= parity & 1;
        if (parity_value)
            setPinHigh;
        else
            setPinLow;
        _delay_loop_2(delay);
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

    if (half_duplex_)
        DDRB &= ~_BV(DDB4);

    SREG = oldSREG;
    _delay_loop_2(delay_stop);
}

void SoftSerial::setTimeout(uint16_t timeout)
{
    timeout_ = timeout * US_TO_COMP(256);
}

uint16_t SoftSerial::timestamp() 
{
    return (uint16_t)(micros() - ts) + timeout_ / US_TO_COMP(256);
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

    PCICR |= _BV(PCIEx);    // ENABLE PCINT
    PCMSKx |= _BV(PCINTxn); // PCINT MASK

    inverted_ = format & 0x40;
    half_duplex_ = format & 0x80;
    stop_bits_ = ((format & 0x8) >> 3) + 1;
    parity_ = (format & 0x30) >> 4;

    if (!half_duplex_)
    {
        DDRB |= _BV(DDB4);
        if (inverted_)
            setPinLow;
        else
            setPinHigh;
    }
    else if (!inverted_)
        PORTx |= _BV(PORTxn); // RX PULLUP for half duplex for non inverted signal. For inverted signal, an external pull down resistor maybe needed. Not needed if the other side has one

    // 1 bit delay in 4 clock cycles
    uint16_t delay = (F_CPU / baud) / 4;
    // substract overheads
    tx_delay = subs(delay, (17 + 2) / 4); //15
    tx_delay_stop = subs(delay * stop_bits_, 17 / 4);
    rx_delay = subs(delay, 15 / 4); //23
    rx_delay_centering = subs(delay / 2, (4 + 4 + 75 + 17 - 15) / 4);
    rx_delay_stop = subs(delay * 3 * stop_bits_ / 4, (37 + 11 + 12) / 4);

    // Set TMR2 to measure ms (max 16Mhz 4ms, 8Mhz 8ms) - shared with sbus
    TCCR2B = _BV(CS22) | _BV(CS21); // SCALER 256
    TCCR2A = 0;
}

#endif