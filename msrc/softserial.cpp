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

void SoftSerial::TIMER0_COMPA_handler()
{
  cont++;
  if (timeout_ / (uint8_t)(255 * COMP_TO_MS(64)) == cont)
  {
    timedout = true;
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

void SoftSerial::PCINT2_handler()
{
  if ( inverted_ ? bit_is_set(PINx, PINxn): !bit_is_set(PINx, PINxn) )
  {
    PCMSKx &= ~_BV(PCINTxn);
    uint8_t incomingByte = 0;

    // start bit
    _delay_loop_2(rx_delay_centering);

    // data
    for (uint8_t i = 0; i < 8; i++)
    {
      
      _delay_loop_2(rx_delay);
      if ( bit_is_set(PINx, PINxn) )
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
    PCIFR = B111;
    PCMSKx |= _BV(PCINTxn);
    if (timedout)
    {
      reset();
      timedout = false;
    }
    writeRx(incomingByte);
    cont = 0;
    OCR0A = TCNT0 - 1;
    TIFR0 |= _BV(OCF0A);
    TIMSK0 |= _BV(OCIE0A);
  }
}

void SoftSerial::initWrite()
{
  uint8_t outgoingByte = readTx();
  if (inverted_)
    outgoingByte = ~outgoingByte;
  PCMSKx &= ~_BV(PCINTxn);

  // start bit
  if (inverted_)
  {
    setPinHigh;
  }
  else
  {
    setPinLow;
  }
  _delay_loop_2(tx_delay);

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
    _delay_loop_2(tx_delay);
    outgoingByte >>= 1;
  }

  // stop bit
  if (inverted_)
  {
    setPinLow;
  }
  else
  {
    setPinHigh;
  }
  _delay_loop_2(tx_delay);

  PCIFR = B111;
  PCMSKx |= _BV(PCINTxn);
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
  tx_delay = subs(delay, 15 / 4); // 15
  rx_delay = subs(delay, 17 / 4); // 23:  16-19  17
  rx_delay_centering = subs(delay / 2, (4 + 4 + 75 + 17 - 18) / 4);
  rx_delay_stop = subs(delay * 3 / 4, (37 + 11) / 4);
}

uint16_t SoftSerial::subs(uint16_t val1, uint16_t val2)
{
    if (val1 > val2)
      return val1 - val2;
    return 1;
}

SoftSerial softSerial;

#endif