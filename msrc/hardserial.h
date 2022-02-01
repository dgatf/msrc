#ifndef HARDSERIAL_H
#define HARDSERIAL_H

#include <Arduino.h>
#include "serial.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)

#if defined(__AVR_ATmega32U4__)
#define UDREx UDRE1
#define UDRIEx UDRIE1
#define U2Xx U2X1
#define RXCIEx RXCIE1
#define TXCIEx TXCIE1
#define TXCx TXC1
#define RXENx RXEN1
#define TXENx TXEN1
#define RXCx RXC1
#else
#define UDREx UDRE0
#define UDRIEx UDRIE0
#define U2Xx U2X0
#define RXCIEx RXCIE0
#define TXCIEx TXCIE0
#define TXCx TXC0
#define RXENx RXEN0
#define TXENx TXEN0
#define RXCx RXC0
#endif

#define SERIAL_8N1 0x06
#define SERIAL_8E1 0x26
#define SERIAL_8N1_RXINV_TXINV 0x46  // val + 0x40
#define SERIAL_HALF_DUP 0x80

#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

class HardSerial : public AbstractSerial
{
private:
public:
    uint8_t timeout_;
    volatile uint16_t ts = 0;
    bool half_duplex_;
    volatile uint8_t *const udr_;
    volatile uint8_t *const ucsra_;
    volatile uint8_t *const ucsrb_;
    volatile uint8_t *const ucsrc_;
    volatile uint8_t *const ubrrl_;
    volatile uint8_t *const ubrrh_;
    volatile uint8_t *const ddr_;
    volatile uint8_t *const port_;
    uint8_t pinRx_;
    uint8_t pinTx_;
    HardSerial(volatile uint8_t *udr, volatile uint8_t *ucsra, volatile uint8_t *ucsrb, volatile uint8_t *ucsrc, volatile uint8_t *ubrrl, volatile uint8_t *ubrrh, volatile uint8_t *ddr, volatile uint8_t *port, uint8_t pinRx, uint8_t pinTx);
    void begin(uint32_t baud, uint8_t format);
    void begin(uint32_t baud) { begin(baud, SERIAL_8N1); }
    void initWrite();
    uint8_t availableTimeout();
    void setTimeout(uint8_t timeout);
    void USART_RX_handler();
    void USART_TX_handler();
    void USART_UDRE_handler();
    uint16_t timestamp() { return ts; }
};

#if defined(UBRRH) || defined(UBRR0H)
extern HardSerial hardSerial0;
#endif
#if defined(UBRR1H)
extern HardSerial hardSerial1;
#endif
#if defined(UBRR2H)
extern HardSerial hardSerial2;
#endif
#if defined(UBRR3H)
extern HardSerial hardSerial3;
#endif

#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

#define SERIAL_8N1 0x00
#define SERIAL_8E1 0x06
#define SERIAL_8N1_RXINV_TXINV 0x30
#define SERIAL_HALF_DUP 0x40

#define SERIAL_7E1 0x02
#define SERIAL_7O1 0x03
#define SERIAL_8O1 0x07
#define SERIAL_7E1_RXINV 0x12
#define SERIAL_7O1_RXINV 0x13
#define SERIAL_8E1_RXINV 0x16
#define SERIAL_8O1_RXINV 0x17
#define SERIAL_7E1_TXINV 0x22
#define SERIAL_7O1_TXINV 0x23
#define SERIAL_8N1_TXINV 0x20
#define SERIAL_8E1_TXINV 0x26
#define SERIAL_8O1_TXINV 0x27
#define SERIAL_7E1_RXINV_TXINV 0x32
#define SERIAL_7O1_RXINV_TXINV 0x33
#define SERIAL_8E1_RXINV_TXINV 0x36
#define SERIAL_8O1_RXINV_TXINV 0x37

#define BAUD2DIVISOR(baud) (((F_PLL / 2 / 16) + ((baud) >> 1)) / (baud))
#define BAUD2DIVISOR2(baud) (((F_BUS / 16) + ((baud) >> 1)) / (baud))
#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest

class HardSerial : public AbstractSerial
{
private:
public:
    uint8_t timeout_;
    volatile uint16_t ts = 0;
    volatile uint32_t *const core_pin_rx_config_;
    volatile uint32_t *const core_pin_tx_config_;
    volatile uint8_t *const uart_d_;
    volatile uint8_t *const uart_s1_;
    volatile uint8_t *const uart_s2_;
    volatile uint8_t *const uart_bdh_;
    volatile uint8_t *const uart_bdl_;
    volatile uint8_t *const uart_c1_;
    volatile uint8_t *const uart_c2_;
    volatile uint8_t *const uart_c3_;
    uint8_t const irq_uart_status_;
    uint32_t const sim_scgc4_uart_;
    uint8_t half_duplex_mode = 0;
    HardSerial(volatile uint32_t * core_pin_rx_config, volatile uint32_t * core_pin_tx_config, volatile uint8_t * uart_d, volatile uint8_t * uart_s1, volatile uint8_t * uart_s2, volatile uint8_t * uart_bdh, volatile uint8_t * uart_bdl, volatile uint8_t * uart_c1, volatile uint8_t * uart_c2, volatile uint8_t * uart_c3, uint8_t irq_uart_status, uint32_t sim_scgc4_uart);
    void begin(uint32_t baud, uint8_t format);
    void begin(uint32_t baud) { begin(baud, SERIAL_8N1); }
    void initWrite();
    uint8_t availableTimeout();
    void setTimeout(uint8_t timeout);
    void UART_IRQ_handler();
    uint16_t timestamp() { return ts; }
};

extern HardSerial hardSerial0;
extern HardSerial hardSerial1;
extern HardSerial hardSerial2;

#endif

#endif
