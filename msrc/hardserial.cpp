#include "hardserial.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)

#if defined(USART_RX_vect)
ISR(USART_RX_vect)
{
    hardSerial0.USART_RX_handler();
}
ISR(USART_TX_vect)
{
    hardSerial0.USART_TX_handler();
}
ISR(USART_UDRE_vect)
{
    hardSerial0.USART_UDRE_handler();
}
#elif defined(USART0_RX_vect)
ISR(USART0_RX_vect)
{
    hardSerial0.USART_RX_handler();
}
ISR(USART0_TX_vect)
{
    hardSerial0.USART_TX_handler();
}
ISR(USART0_UDRE_vect)
{
    hardSerial0.USART_UDRE_handler();
}
#endif

#if defined(USART1_RX_vect)
ISR(USART1_RX_vect)
{
    hardSerial1.USART_RX_handler();
}
ISR(USART1_TX_vect)
{
    hardSerial1.USART_TX_handler();
}
ISR(USART1_UDRE_vect)
{
    hardSerial1.USART_UDRE_handler();
}
#endif

#if defined(USART2_RX_vect)
ISR(USART2_RX_vect)
{
    hardSerial2.USART_RX_handler();
}
ISR(USART2_TX_vect)
{
    hardSerial2.USART_TX_handler();
}
ISR(USART2_UDRE_vect)
{
    hardSerial2.USART_UDRE_handler();
}
#endif

#if defined(USART3_RX_vect)
ISR(USART3_RX_vect)
{
    hardSerial3.USART_RX_handler();
}
ISR(USART3_TX_vect)
{
    hardSerial3.USART_TX_handler();
}
ISR(USART3_UDRE_vect)
{
    hardSerial3.USART_UDRE_handler();
}
#endif

void HardSerial::USART_UDRE_handler()
{
    if (availableTx())
        *udr_ = readTx();
    else
    {
        *ucsrb_ &= ~_BV(UDRIEx);
        if (half_duplex_)
        {
            *ucsra_ |= _BV(TXCx);
            *ucsrb_ |= _BV(TXCIEx);
        }
    }
}

void HardSerial::USART_TX_handler()
{
    if (!availableTx())
    {
        *ucsrb_ &= ~_BV(TXCIEx);
        *ucsrb_ &= ~_BV(TXENx);
        *ddr_ &= ~_BV(pinTx_);
        *ucsrb_ |= _BV(RXENx);
    }
}

void HardSerial::USART_RX_handler()
{
    if ((uint16_t)(micros() - ts) > timeout_ && timeout_)
        reset();
    writeRx(*udr_);
    ts = micros();
}

void HardSerial::begin(uint32_t baud, uint8_t format)
{
    half_duplex_ = format & 0x80;
    cli();
    *ucsrb_ = _BV(RXCIEx) | _BV(RXENx);
    if (!half_duplex_)
        *ucsrb_ |= _BV(TXENx);
    else
        *port_ |= _BV(pinRx_); // RX PULLUP
    uint16_t val = (F_CPU / 4 / baud - 1) / 2;
    *ubrrl_ = val;
    *ubrrh_ = val >> 8;
    *ucsra_ = 1 << U2Xx;
    *ucsrc_ = format & ~0xC0;
    sei();
}

void HardSerial::initWrite()
{
    if (half_duplex_)
    {
        *ucsrb_ |= _BV(TXENx);
        *ucsrb_ &= ~_BV(RXENx);
    }
    *ucsrb_ |= _BV(UDREx);
}

uint8_t HardSerial::availableTimeout()
{
    noInterrupts();
    uint16_t tsCopy = ts;
    uint8_t availableCopy = available();
    interrupts();
    if ((uint16_t)(micros() - tsCopy) > timeout_)
        return availableCopy;
    else
    {
        return 0;
    }
}

void HardSerial::setTimeout(uint16_t timeout)
{
    timeout_ = timeout;
}

HardSerial::HardSerial(volatile uint8_t *udr, volatile uint8_t *ucsra, volatile uint8_t *ucsrb, volatile uint8_t *ucsrc, volatile uint8_t *ubrrl, volatile uint8_t *ubrrh, volatile uint8_t *ddr, volatile uint8_t *port, uint8_t pinRx, uint8_t pinTx) : udr_(udr), ucsra_(ucsra), ucsrb_(ucsrb), ucsrc_(ucsrc), ubrrl_(ubrrl), ubrrh_(ubrrh), ddr_(ddr), port_(port), pinRx_(pinRx), pinTx_(pinTx) {}

#if defined(UBRR0H) && defined(__AVR_ATmega2560__)
HardSerial hardSerial0(&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0L, &UBRR0H, &DDRE, &PORTE, 0, 1);
#endif
#if defined(UBRR0H) && (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB))
HardSerial hardSerial0(&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0L, &UBRR0H, &DDRD, &PORTD, 0, 1);
#endif
#if defined(UBRR1H) && (defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__))
HardSerial hardSerial1(&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1L, &UBRR1H, &DDRD, &PORTD, 2, 3);
#endif
#if defined(UBRR1H) && (defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB))
HardSerial hardSerial1(&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1L, &UBRR1H, &DDRB, &PORTB, 3, 4);
#endif
#if defined(UBRR2H)
HardSerial hardSerial2(&UDR2, &UCSR2A, &UCSR2B, &UCSR2C, &UBRR2L, &UBRR2H, &DDRH, &PORTH, 0, 1);
#endif
#if defined(UBRR3H)
HardSerial hardSerial3(&UDR3, &UCSR3A, &UCSR3B, &UCSR3C, &UBRR3L, &UBRR3H, &DDRJ, &PORTJ, 0, 1);
#endif

#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

void uart0_status_isr()
{
    hardSerial0.UART_IRQ_handler();
}

void uart1_status_isr()
{
    hardSerial1.UART_IRQ_handler();
}

void uart2_status_isr()
{
    hardSerial2.UART_IRQ_handler();
}

void HardSerial::UART_IRQ_handler()
{
    if ((*uart_c2_ & UART_C2_RIE) && (*uart_s1_ & UART_S1_RDRF))
    {
        if ((uint16_t)(micros() - ts) > timeout_ && timeout_)
            reset();
        uint8_t c = *uart_d_;
        writeRx(c);
        ts = micros();
    }
    if ((*uart_c2_ & UART_C2_TIE) && (*uart_s1_ & UART_S1_TDRE))
    {
        if (availableTx())
        {
            *uart_d_ = readTx();
        }
        else
        {
            *uart_c2_ &= ~UART_C2_TIE;
            if (half_duplex_mode)
                *uart_c2_ |= UART_C2_TCIE;
        }
    }
    if ((*uart_c2_ & UART_C2_TCIE) && (*uart_s1_ & UART_S1_TC))
    {
        *uart_c3_ &= ~UART_C3_TXDIR;
        *uart_c2_ &= ~UART_C2_TCIE;
        *uart_c2_;
        *uart_d_;
        *uart_c2_ |= UART_C2_RIE;
    }
}

void HardSerial::begin(uint32_t baud, uint8_t format)
{
    // Uart setup

    *core_pin_rx_config_ = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
    *core_pin_tx_config_ = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

    SIM_SCGC4 |= sim_scgc4_uart_;
    uint32_t divisor;
    if (uart_d_ == &UART0_D)
        divisor = BAUD2DIVISOR(baud);
    else
        divisor = BAUD2DIVISOR2(baud);
    if (divisor < 1)
        divisor = 1;
    *uart_bdh_ = (divisor >> 8) & 0x1F;
    *uart_bdl_ = divisor & 0xFF;
    *uart_c1_ = 0;
    *uart_c2_ = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
    NVIC_SET_PRIORITY(irq_uart_status_, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(irq_uart_status_);

    // Format

    // parity b1-2
    uint8_t c = *uart_c1_;
    c = (c & ~0x13) | (format & 0x03);
    *uart_c1_ = c;
    if (format & 0x04)
        c |= 0x10;		// 9 bits (might include parity)
	*uart_c1_ = c;

    // 2 stop bits b8
    if (format & 0x80)
    {
        uint8_t bdl = *uart_bdl_;
        *uart_bdh_ |= UART_BDH_SBNS;
        *uart_bdl_ = bdl;
    }

    // half duplex b7
    if ((format & SERIAL__HALF_DUP) != 0)
    {
        c = *uart_c1_;
        c |= UART_C1_LOOPS | UART_C1_RSRC;
        *uart_c1_ = c;
        half_duplex_mode = 1;
        *core_pin_tx_config_ |= PORT_PCR_PE | PORT_PCR_PS;
    }
    else
        half_duplex_mode = 0;

    // polarity b5-6
    c = *uart_s2_ & ~0x10;
    if (format & 0x10)
    {
        c |= 0x10; // rx invert
        *core_pin_rx_config_ &= ~PORT_PCR_PS;
    }
    *uart_s2_ = c;
    c = *uart_c3_ & ~0x10;
    if (format & 0x20)
    {
        if ((format & 0x0F) == 0x04)
            *uart_c3_ &= ~0x40; // 8N2 is 9 bit with 9th bit always 0 (if inverted)
        c |= 0x10; // tx invert
        if (half_duplex_mode)
            *core_pin_tx_config_ &= ~PORT_PCR_PS;
    }
    else
    {
        if ((format & 0x0F) == 0x04)
            *uart_c3_ |= 0x40; // 8N2 is 9 bit with 9th bit always 1
    }
    *uart_c3_ = c;
}

void HardSerial::initWrite()
{
    *uart_c2_ |= UART_C2_TIE;
    if (half_duplex_mode)
    {
        *uart_c2_ &= ~UART_C2_RIE;
        *uart_c3_ |= UART_C3_TXDIR;
    }
}

uint8_t HardSerial::availableTimeout()
{
    noInterrupts();
    uint16_t tsCopy = ts;
    uint8_t availableCopy = available();
    interrupts();
    if ((uint16_t)(micros() - tsCopy) > timeout_)
        return availableCopy;
    else
        return 0;
}

void HardSerial::setTimeout(uint16_t timeout)
{
    timeout_ = timeout;
}

HardSerial::HardSerial(volatile uint32_t *core_pin_rx_config, volatile uint32_t *core_pin_tx_config, volatile uint8_t *uart_d, volatile uint8_t *uart_s1, volatile uint8_t *uart_s2, volatile uint8_t *uart_bdh, volatile uint8_t *uart_bdl, volatile uint8_t *uart_c1, volatile uint8_t *uart_c2, volatile uint8_t *uart_c3, uint8_t irq_uart_status, uint32_t sim_scgc4_uart) : core_pin_rx_config_(core_pin_rx_config), core_pin_tx_config_(core_pin_tx_config), uart_d_(uart_d), uart_s1_(uart_s1), uart_s2_(uart_s2), uart_bdh_(uart_bdh), uart_bdl_(uart_bdl), uart_c1_(uart_c1), uart_c2_(uart_c2), uart_c3_(uart_c3), irq_uart_status_(irq_uart_status), sim_scgc4_uart_(sim_scgc4_uart) {}

HardSerial hardSerial0(&CORE_PIN0_CONFIG, &CORE_PIN1_CONFIG, &UART0_D, &UART0_S1, &UART0_S2, &UART0_BDH, &UART0_BDL, &UART0_C1, &UART0_C2, &UART0_C3, IRQ_UART0_STATUS, SIM_SCGC4_UART0);

HardSerial hardSerial1(&CORE_PIN9_CONFIG, &CORE_PIN10_CONFIG, &UART1_D, &UART1_S1, &UART1_S2, &UART1_BDH, &UART1_BDL, &UART1_C1, &UART1_C2, &UART1_C3, IRQ_UART1_STATUS, SIM_SCGC4_UART1);

HardSerial hardSerial2(&CORE_PIN7_CONFIG, &CORE_PIN8_CONFIG, &UART2_D, &UART2_S1, &UART2_S2, &UART2_BDH, &UART2_BDL, &UART2_C1, &UART2_C2, &UART2_C3, IRQ_UART2_STATUS, SIM_SCGC4_UART2);

#endif
