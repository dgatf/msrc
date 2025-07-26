/*
 * Copyright (c) 2022, Daniel Gorbea
 * All rights reserved.
 *
 * This source code is licensed under the MIT-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Library for pin capture timer for RP2040
 */

#include "capture_edge.h"

#include <stdio.h>

#include "hardware/dma.h"
#include "hardware/irq.h"

#define MAX_PIN_COUNT 2

static uint pins_, counter_;
static const uint reload_counter_ = 0xffffffff, reload_pins_ = 1;
static uint sm_, offset_, dma_channel_write_pins_, dma_channel_write_counter_, dma_channel_counter_,
    dma_channel_reload_counter_, dma_channel_reload_pins_, dma_channel_reload_write_counter_, pin_count_;
static PIO pio_;
static void (*handler_[MAX_PIN_COUNT])(uint counter, edge_type_t edge) = {NULL};

static inline void handler_pio(void);
static inline edge_type_t get_captured_edge(uint pin, uint pins, uint prev);
static inline uint bit_value(uint pos);

void capture_edge_init(PIO pio, uint pin_base, uint pin_count, float clk_div, uint irq) {
    pio_ = pio;
    pin_count_ = pin_count;

    // pio capture
    sm_ = pio_claim_unused_sm(pio_, true);
    offset_ = pio_add_program(pio_, &capture_edge_program);
    pio_sm_set_consecutive_pindirs(pio_, sm_, pin_base, pin_count, false);
    pio_sm_config c = capture_edge_program_get_default_config(offset_);
    sm_config_set_clkdiv(&c, clk_div);
    sm_config_set_in_pins(&c, pin_base);
    pio->instr_mem[offset_ + 3] = pio_encode_in(pio_pins, pin_count);
    pio->instr_mem[offset_ + 4] = pio_encode_in(pio_null, 32 - pin_count);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    if (irq == PIO0_IRQ_0 || irq == PIO1_IRQ_0)
        pio_set_irq0_source_enabled(pio_, (enum pio_interrupt_source)(pis_interrupt0 + CAPTURE_EDGE_IRQ_NUM), true);
    else
        pio_set_irq1_source_enabled(pio_, (enum pio_interrupt_source)(pis_interrupt0 + CAPTURE_EDGE_IRQ_NUM), true);
    pio_interrupt_clear(pio_, CAPTURE_EDGE_IRQ_NUM);
    pio_sm_init(pio_, sm_, offset_ + capture_edge_offset_start, &c);
    irq_set_exclusive_handler(irq, handler_pio);
    irq_set_enabled(irq, true);

    // get dma channels
    dma_channel_write_pins_ = dma_claim_unused_channel(true);
    dma_channel_write_counter_ = dma_claim_unused_channel(true);
    dma_channel_counter_ = dma_claim_unused_channel(true);
    dma_channel_reload_counter_ = dma_claim_unused_channel(true);
    dma_channel_reload_pins_ = dma_claim_unused_channel(true);
    dma_channel_reload_write_counter_ = dma_claim_unused_channel(true);

    // dma channel write pins
    dma_channel_config config_dma_channel_write_pins = dma_channel_get_default_config(dma_channel_write_pins_);
    channel_config_set_transfer_data_size(&config_dma_channel_write_pins, DMA_SIZE_32);
    channel_config_set_write_increment(&config_dma_channel_write_pins, false);
    channel_config_set_read_increment(&config_dma_channel_write_pins, false);
    channel_config_set_dreq(&config_dma_channel_write_pins, pio_get_dreq(pio_, sm_, false));
    channel_config_set_chain_to(&config_dma_channel_write_pins, dma_channel_reload_pins_);
    dma_channel_configure(dma_channel_write_pins_, &config_dma_channel_write_pins,
                          &pins_,           // write address
                          &pio_->rxf[sm_],  // read address
                          1, false);

    // dma channel write counter
    dma_channel_config config_dma_channel_write_counter = dma_channel_get_default_config(dma_channel_write_counter_);
    channel_config_set_transfer_data_size(&config_dma_channel_write_counter, DMA_SIZE_32);
    channel_config_set_write_increment(&config_dma_channel_write_counter, false);
    channel_config_set_read_increment(&config_dma_channel_write_counter, false);
    channel_config_set_dreq(&config_dma_channel_write_pins, pio_get_dreq(pio_, sm_, false));
    channel_config_set_chain_to(&config_dma_channel_write_counter, dma_channel_reload_write_counter_);
    dma_channel_configure(dma_channel_write_counter_, &config_dma_channel_write_counter,
                          &counter_,                                         // write address
                          &dma_hw->ch[dma_channel_counter_].transfer_count,  // read address
                          1, false);

    // dma channel counter
    dma_channel_config config_dma_channel_counter = dma_channel_get_default_config(dma_channel_counter_);
    channel_config_set_write_increment(&config_dma_channel_counter, false);
    channel_config_set_read_increment(&config_dma_channel_counter, false);
    uint dma_timer = dma_claim_unused_timer(true);
    dma_timer_set_fraction(dma_timer, 1, COUNTER_CYCLES);
    channel_config_set_dreq(&config_dma_channel_counter, dma_get_timer_dreq(dma_timer));
    channel_config_set_chain_to(&config_dma_channel_counter, dma_channel_reload_counter_);
    dma_channel_configure(dma_channel_counter_, &config_dma_channel_counter,
                          NULL,  // write address
                          NULL,  // read address
                          0xffffffff, false);

    // dma channel reload counter
    dma_channel_config config_dma_channel_reload_counter = dma_channel_get_default_config(dma_channel_reload_counter_);
    channel_config_set_transfer_data_size(&config_dma_channel_reload_counter, DMA_SIZE_32);
    channel_config_set_write_increment(&config_dma_channel_reload_counter, false);
    channel_config_set_read_increment(&config_dma_channel_reload_counter, false);
    dma_channel_configure(dma_channel_reload_counter_, &config_dma_channel_reload_counter,
                          &dma_hw->ch[dma_channel_counter_].al1_transfer_count_trig,  // write address
                          &reload_counter_,                                           // read address
                          1, false);

    // dma channel reload pins
    dma_channel_config config_dma_channel_reload_pins = dma_channel_get_default_config(dma_channel_reload_pins_);
    channel_config_set_transfer_data_size(&config_dma_channel_reload_pins, DMA_SIZE_32);
    channel_config_set_write_increment(&config_dma_channel_reload_pins, false);
    channel_config_set_read_increment(&config_dma_channel_reload_pins, false);
    dma_channel_configure(dma_channel_reload_pins_, &config_dma_channel_reload_pins,
                          &dma_hw->ch[dma_channel_write_pins_].al1_transfer_count_trig,  // write address
                          &reload_pins_,                                                 // read address
                          1, false);

    // dma channel reload write counter
    dma_channel_config config_dma_channel_reload_write_counter =
    dma_channel_get_default_config(dma_channel_reload_write_counter_);
    channel_config_set_transfer_data_size(&config_dma_channel_reload_write_counter, DMA_SIZE_32);
    channel_config_set_write_increment(&config_dma_channel_reload_write_counter, false);
    channel_config_set_read_increment(&config_dma_channel_reload_write_counter, false);
    dma_channel_configure(dma_channel_reload_write_counter_, &config_dma_channel_reload_write_counter,
                          &dma_hw->ch[dma_channel_write_counter_].al1_transfer_count_trig,  // write address
                          &reload_pins_,                                                    // read address
                          1, false);

    dma_start_channel_mask((1 << dma_channel_write_pins_) | (1 << dma_channel_write_counter_) |
                           (1 << dma_channel_counter_));
    pio_sm_set_enabled(pio_, sm_, true);
}

void capture_edge_set_handler(uint pin, capture_handler_t handler) {
    if (pin < pin_count_) {
        handler_[pin] = handler;
    }
}

void capture_edge_remove(void) {
    for (uint pin = 0; pin < pin_count_; pin++) capture_edge_set_handler(pin, NULL);
    pio_remove_program(pio_, &capture_edge_program, offset_);
    pio_sm_unclaim(pio_, sm_);
    dma_channel_unclaim(dma_channel_write_pins_);
    dma_channel_unclaim(dma_channel_write_counter_);
    dma_channel_unclaim(dma_channel_counter_);
    dma_channel_unclaim(dma_channel_reload_counter_);
    dma_channel_unclaim(dma_channel_reload_pins_);
    dma_channel_unclaim(dma_channel_reload_write_counter_);
}

static inline void handler_pio(void) {
    static uint prev_pins = 0;
    uint counter = ~counter_;
    uint pins = pins_;
    for (uint pin = 0; pin < pin_count_; pin++) {
        edge_type_t edge = get_captured_edge(pin, pins, prev_pins);
        if (edge && *handler_[pin]) handler_[pin](counter, edge);
    }
    prev_pins = pins;
    pio_interrupt_clear(pio_, CAPTURE_EDGE_IRQ_NUM);
}

static inline edge_type_t get_captured_edge(uint pin, uint pins, uint prev) {
    if ((bit_value(pin) & pins) ^ (bit_value(pin) & prev) && (bit_value(pin) & pins)) return EDGE_RISE;
    if ((bit_value(pin) & pins) ^ (bit_value(pin) & prev) && !(bit_value(pin) & pins)) return EDGE_FALL;
    return EDGE_NONE;
}

static inline uint bit_value(uint pos) { return 1 << pos; }
