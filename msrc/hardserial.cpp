#include "hardserial.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)

#if defined(UBRR0H) && defined(__AVR_ATmega328P__)
ISR(USART_RX_vect)
{
    hardSerial0.USART_RX_handler();
}
ISR(USART_UDRE_vect)
{
    hardSerial0.USART_UDRE_handler();
}
#endif

#if defined(UBRR0H) && !defined(__AVR_ATmega328P__)
ISR(USART0_RX_vect)
{
    hardSerial0.USART_RX_handler();
}
ISR(USART0_UDRE_vect)
{
    hardSerial0.USART_UDRE_handler();
}
#endif

#if defined(UBRR1H)
ISR(USART1_RX_vect)
{
    hardSerial1.USART_RX_handler();
}
ISR(USART1_UDRE_vect)
{
    hardSerial1.USART_UDRE_handler();
}
#endif

#if defined(UBRR2H)
ISR(USART2_RX_vect)
{
    hardSerial2.USART_RX_handler();
}
ISR(USART2_UDRE_vect)
{
    hardSerial2.USART_UDRE_handler();
}
#endif

#if defined(UBRR3H)
ISR(USART3_RX_vect)
{
    hardSerial3.USART_RX_handler();
}
ISR(USART3_UDRE_vect)
{
    hardSerial3.USART_UDRE_handler();
}
#endif

void HardSerial::USART_UDRE_handler()
{
    if (buffTx.available())
        *udr_ = buffTx.read();
    else
        *ucsrb_ &= ~_BV(UDREx);
}

void HardSerial::USART_RX_handler()
{
    if (timeout_)
    {
        if ((uint16_t)micros() - ts > timeout_ * 1000)
        {
            buffRx.reset();
        }
    }
    buffRx.write(*udr_);
    ts = micros();
}

void HardSerial::begin(uint32_t baud, uint8_t format)
{
    cli();
    *ucsrb_ = _BV(RXCIEx) | _BV(RXENx) | _BV(TXENx);
    uint16_t val = (F_CPU / 4 / baud - 1) / 2;
    *ubrrl_ = val;
    *ubrrh_ = val >> 8;
    *ucsra_ = 1 << U2Xx;
    *ucsrc_ = format & ~0x40;
    sei();
}

void HardSerial::initWrite()
{
    *ucsrb_ |= _BV(UDREx);
}

HardSerial::HardSerial(volatile uint8_t *udr, volatile uint8_t *ucsra, volatile uint8_t *ucsrb, volatile uint8_t *ucsrc, volatile uint8_t *ubrrl, volatile uint8_t *ubrrh)
    : udr_(udr), ucsra_(ucsra), ucsrb_(ucsrb), ucsrc_(ucsrc), ubrrl_(ubrrl), ubrrh_(ubrrh) {}

#if defined(UBRR0H)
HardSerial hardSerial0(&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0L, &UBRR0H);
#endif
#if defined(UBRR1H)
HardSerial hardSerial1(&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1L, &UBRR1H);
#endif
#if defined(UBRR2H)
HardSerial hardSerial2(&UDR2, &UCSR2A, &UCSR2B, &UCSR2C, &UBRR2L, &UBRR2H);
#endif
#if defined(UBRR3H)
HardSerial hardSerial3(&UDR3, &UCSR3A, &UCSR3B, &UCSR3C, &UBRR3L, &UBRR3H);
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
    if (*uart_s1_ & UART_S1_RDRF) // Rx complete interrupt
    {
        if (timeout_)
        {
            if ((uint16_t)micros() - ts > timeout_ * 1000)
            {
                buffRx.reset();
            }
        }
        uint8_t c = *uart_d_;
        buffRx.write(c);
        ts = micros();
    }
    if (*uart_s1_ & UART_S1_TDRE) // Tx complete interrupt
    {
        if (buffTx.available())
        {
            if (half_duplex_mode)
            {
                __disable_irq();
                volatile uint32_t reg = *uart_c3_;
                reg |= UART_C3_TXDIR;
                *uart_c3_ = reg;
                __enable_irq();
            }
            *uart_d_ = buffTx.read();
        }
        else
        {
            *uart_c2_ &= ~UART_C2_TIE;
            if (half_duplex_mode)
            {
                __disable_irq();
                volatile uint32_t reg = UART0_C3;
                reg &= ~UART_C3_TXDIR;
                UART0_C3 = reg;
                __enable_irq();
            }
        }
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
        divisor = BAUD2DIV(baud);
    else
        divisor = BAUD2DIV2(baud);
    if (divisor < 1)
        divisor = 1;
    *uart_bdh_ = (divisor >> 8) & 0x1F;
    *uart_bdl_ = divisor & 0xFF;
    *uart_c1_ = 0;
    *uart_c2_ = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
    NVIC_SET_PRIORITY(irq_uart_status_, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(irq_uart_status_);

    // Format

    // parity
    uint8_t c = *uart_c1_;
    c = (c & ~0x13) | (format & 0x03); // configure parity
    *uart_c1_ = c;

    // polarity
    c = *uart_s2_ & ~0x10;
    if (format & 0x10)
        c |= 0x10; // rx invert
    *uart_s2_ = c;
    c = *uart_c3_ & ~0x10;
    if (format & 0x20)
        c |= 0x10; // tx invert
    *uart_c3_ = c;

    // half duplex.
    if ((format & SERIAL_HALF_DUP) != 0)
    {
        c = *uart_c1_;
        c |= UART_C1_LOOPS | UART_C1_RSRC;
        *uart_c1_ = c;
        half_duplex_mode = 1;
    }
    else
        half_duplex_mode = 0;
}

void HardSerial::initWrite()
{
    *uart_c2_ |= UART_C2_TIE;
}

HardSerial::HardSerial(volatile uint32_t *core_pin_rx_config, volatile uint32_t *core_pin_tx_config, volatile uint8_t *uart_d, volatile uint8_t *uart_s1, volatile uint8_t *uart_s2, volatile uint8_t *uart_bdh, volatile uint8_t *uart_bdl, volatile uint8_t *uart_c1, volatile uint8_t *uart_c2, volatile uint8_t *uart_c3, uint8_t irq_uart_status, uint32_t sim_scgc4_uart) : core_pin_rx_config_(core_pin_rx_config), core_pin_tx_config_(core_pin_tx_config), uart_d_(uart_d), uart_s1_(uart_s1), uart_s2_(uart_s2), uart_bdh_(uart_bdh), uart_bdl_(uart_bdl), uart_c1_(uart_c1), uart_c2_(uart_c2), uart_c3_(uart_c3), irq_uart_status_(irq_uart_status), sim_scgc4_uart_(sim_scgc4_uart) {}

HardSerial hardSerial0(&CORE_PIN0_CONFIG, &CORE_PIN1_CONFIG, &UART0_D, &UART0_S1, &UART0_S2, &UART0_BDH, &UART0_BDL, &UART0_C1, &UART0_C2, &UART0_C3, IRQ_UART0_STATUS, SIM_SCGC4_UART0);

HardSerial hardSerial1(&CORE_PIN9_CONFIG, &CORE_PIN10_CONFIG, &UART1_D, &UART1_S1, &UART1_S2, &UART1_BDH, &UART1_BDL, &UART1_C1, &UART1_C2, &UART1_C3, IRQ_UART1_STATUS, SIM_SCGC4_UART1);

HardSerial hardSerial2(&CORE_PIN7_CONFIG, &CORE_PIN8_CONFIG, &UART2_D, &UART2_S1, &UART2_S2, &UART2_BDH, &UART2_BDL, &UART2_C1, &UART2_C2, &UART2_C3, IRQ_UART2_STATUS, SIM_SCGC4_UART2);

#endif
