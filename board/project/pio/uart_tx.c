#include "uart_tx.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

static uint sm_, offset_;
static PIO pio_;
static uint8_t data_bits_;
static uint8_t parity_;

uint uart_tx_init(PIO pio, uint pin, uint baudrate, uint data_bits, uint stop_bits, uint parity) {
    pio_ = pio;
    data_bits_ = data_bits;
    parity_ = parity;
    gpio_pull_up(pin);
    sm_ = pio_claim_unused_sm(pio_, true);
    pio_gpio_init(pio_, pin);
    pio_sm_set_consecutive_pindirs(pio_, sm_, pin, 1, false);
    pio_sm_set_pins(pio, sm_, 0xFFFF);
    offset_ = pio_add_program(pio_, &uart_tx_program);
    pio_->instr_mem[offset_] = pio_encode_set(pio_x, parity == UART_PARITY_NONE ? data_bits - 1 : data_bits);
    pio_->instr_mem[offset_ + 6] = pio_encode_set(pio_pins, 1) | pio_encode_delay(16 * stop_bits - 1);
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

void uart_tx_write(uint32_t c) {
    if (parity_ != UART_PARITY_NONE) {
        bool parity_bit = 0;
        for (uint i = 0; i < data_bits_; i++) {
            parity_bit ^= (c >> i) & 1;
        }
        c |= (parity_bit << (data_bits_));
        if (parity_ == UART_PARITY_ODD) {
            c ^= (1 << (data_bits_));
        }
    }
    pio_sm_put_blocking(pio_, sm_, c);
}

void uart_tx_write_bytes(void *data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        switch (data_bits_) {
            case 1 ... 8:
                uart_tx_write(((uint8_t *)data)[i]);
                break;
            case 9 ... 16:
                uart_tx_write(((uint16_t *)data)[i]);
                break;
            case 17 ... 32:
                uart_tx_write(((uint32_t *)data)[i]);
                break;
        }
    }
}

void uart_tx_remove(void) {
    pio_remove_program(pio_, &uart_tx_program, offset_);
    pio_sm_unclaim(pio_, sm_);
}