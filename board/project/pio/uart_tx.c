#include "uart_tx.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

static uint sm_, offset_;
static PIO pio_;

uint uart_tx_init(PIO pio, uint pin, uint baudrate) {
    pio_ = pio;
    sm_ = pio_claim_unused_sm(pio_, true);
    pio_sm_set_consecutive_pindirs(pio_, sm_, pin, 1, true);
    pio_gpio_init(pio_, pin);
    gpio_pull_up(pin);
    offset_ = pio_add_program(pio_, &uart_tx_program);
    pio_sm_config c = uart_tx_program_get_default_config(offset_);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    float div = (float)clock_get_hz(clk_sys) / (UART_TX_CYCLES_PER_BIT * baudrate);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio_, sm_, offset_, &c);
    pio_sm_set_enabled(pio_, sm_, true);
    return sm_;
}

void uart_tx_write(uint8_t c) { pio_sm_put_blocking(pio_, sm_, c); }

void uart_tx_write_bytes(uint8_t *data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) uart_tx_write(data[i]);
}

void uart_tx_remove(void) {
    pio_remove_program(pio_, &uart_tx_program, offset_);
    pio_sm_unclaim(pio_, sm_);
}