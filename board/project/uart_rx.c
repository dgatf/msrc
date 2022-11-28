#include "uart_rx.h"

static uint sm_;
static PIO pio_;
static void (*handler_)(uint8_t data) = NULL;

static inline void handler_pio();

uint uart_rx_init(PIO pio, uint pin, uint baudrate, uint irq)
{
    pio_ = pio;
    sm_ = pio_claim_unused_sm(pio_, true);

    pio_sm_set_consecutive_pindirs(pio_, sm_, pin, 1, false);
    pio_gpio_init(pio_, pin);

    uint offset = pio_add_program(pio_, &uart_rx_program);
    pio_sm_config c = uart_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    float div = (float)clock_get_hz(clk_sys) / (8 * baudrate);
    sm_config_set_clkdiv(&c, div);

    if (irq == PIO0_IRQ_0 || irq == PIO1_IRQ_0)
        pio_set_irq0_source_enabled(pio_, (enum pio_interrupt_source)(pis_interrupt0 + UART_RX_IRQ_NUM), true);
    else
        pio_set_irq1_source_enabled(pio_, (enum pio_interrupt_source)(pis_interrupt0 + UART_RX_IRQ_NUM), true);
    pio_interrupt_clear(pio_, UART_RX_IRQ_NUM);

    pio_sm_init(pio_, sm_, offset, &c);
    pio_sm_set_enabled(pio_, sm_, true);
    irq_set_exclusive_handler(irq, handler_pio);
    irq_set_enabled(irq, true);

    return sm_;
}

void uart_rx_set_handler(uart_rx_handler_t handler)
{
    handler_ = handler;
}

static inline void handler_pio()
{
    pio_interrupt_clear(pio_, UART_RX_IRQ_NUM);
    if (handler_)
    {
        uint data = pio_sm_get_blocking(pio_, sm_);
        handler_(data >> 24);
    }
}