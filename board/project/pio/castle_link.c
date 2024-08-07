#include "castle_link.h"

#include <math.h>

#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

static uint sm_pulse_, sm_counter_, offset_pulse_, offset_counter_;
static PIO pio_;
static void (*handler_)(castle_link_telemetry_t packet) = {NULL};

static inline void handler_pio();

void castle_link_init(PIO pio, uint pin, uint irq) {
    pio_ = pio;
    sm_pulse_ = pio_claim_unused_sm(pio_, true);
    offset_pulse_ = pio_add_program(pio_, &pulse_program);
    pio_gpio_init(pio_, pin + 1);
    pio_sm_set_consecutive_pindirs(pio_, sm_pulse_, pin, 2, false);
    pio_sm_set_pins(pio, sm_pulse_, 0);

    pio_sm_config c_pulse = pulse_program_get_default_config(offset_pulse_);
    sm_config_set_clkdiv(&c_pulse, 10);
    sm_config_set_in_pins(&c_pulse, pin);
    sm_config_set_set_pins(&c_pulse, pin + 1, 1);

    pio_sm_init(pio_, sm_pulse_, offset_pulse_, &c_pulse);
    pio_sm_set_enabled(pio_, sm_pulse_, true);

    sm_counter_ = pio_claim_unused_sm(pio_, true);
    offset_counter_ = pio_add_program(pio_, &counter_program);

    pio_sm_config c_counter = counter_program_get_default_config(offset_counter_);
    sm_config_set_clkdiv(&c_counter, 5);
    sm_config_set_in_pins(&c_counter, pin);
    if (irq == PIO0_IRQ_0 || irq == PIO1_IRQ_0)
        pio_set_irq0_source_enabled(pio_, (enum pio_interrupt_source)(pis_interrupt0 + CASTLE_LINK_IRQ_NUM), true);
    else
        pio_set_irq1_source_enabled(pio_, (enum pio_interrupt_source)(pis_interrupt0 + CASTLE_LINK_IRQ_NUM), true);
    pio_interrupt_clear(pio_, CASTLE_LINK_IRQ_NUM);
    pio_sm_init(pio_, sm_counter_, offset_counter_ + counter_offset_start, &c_counter);
    pio_sm_set_enabled(pio_, sm_counter_, true);
    irq_set_exclusive_handler(irq, handler_pio);
    irq_set_enabled(irq, true);
}

void castle_link_set_handler(castle_link_handler_t handler) { handler_ = handler; }

void castle_link_remove() {
    castle_link_set_handler(NULL);
    pio_remove_program(pio_, &pulse_program, offset_pulse_);
    pio_remove_program(pio_, &counter_program, offset_counter_);
    pio_sm_unclaim(pio_, sm_pulse_);
    pio_sm_unclaim(pio_, sm_counter_);
}

static inline void handler_pio() {
    static uint index = 0;
    static const float scaler[11] = {0, 20, 4, 50, 1, 0.2502, 20416.7, 4, 4, 30, 63.8125};
    static uint value[12];

    pio_interrupt_clear(pio_, CASTLE_LINK_IRQ_NUM);
    if (pio_sm_is_rx_fifo_full(pio_, sm_counter_)) {
        pio_sm_clear_fifos(pio_, sm_counter_);
        return;
    }
    uint data = pio_sm_get_blocking(pio_, sm_counter_);
    if (data > 50000) {
        index = 0;
        // printf("%i \n", data);
        return;
    }
    if (index > 10) return;
    value[index] = data;
    // printf("(%u)%u ", index, value[index]);
    if (index == 10) {
        uint calibration;
        castle_link_telemetry_t packet;
        if (value[9] < value[10]) {
            calibration = value[0] / 2 + value[9];
            packet.is_temp_ntc = true;
            float temp_raw = ((float)value[10] - calibration / 2) * scaler[10] / calibration;
            packet.temperature =
                1 / (log(temp_raw * 10200.0 / (255.0 - temp_raw) / 10000.0) / 3455.0 + 1.0 / 298.0) - 273.0;
        } else {
            calibration = value[0] / 2 + value[10];
            packet.is_temp_ntc = false;
            packet.temperature = ((float)value[9] - calibration / 2) * scaler[9] / calibration;
        }
        packet.voltage = ((float)value[1] - calibration / 2) * scaler[1] / calibration;
        packet.ripple_voltage = ((float)value[2] - calibration / 2) * scaler[2] / calibration;
        packet.current = ((float)value[3] - calibration / 2) * scaler[3] / calibration;
        packet.thr = ((float)value[4] - calibration / 2) * scaler[4] / calibration;
        packet.output = ((float)value[5] - calibration / 2) * scaler[5] / calibration;
        packet.rpm = ((float)value[6] - calibration / 2) * scaler[6] / calibration;
        packet.voltage_bec = ((float)value[7] - calibration / 2) * scaler[7] / calibration;
        packet.current_bec = ((float)value[8] - calibration / 2) * scaler[8] / calibration;
        if (packet.voltage < 0) packet.voltage = 0;
        if (packet.ripple_voltage < 0) packet.ripple_voltage = 0;
        if (packet.current < 0) packet.current = 0;
        if (packet.thr < 0) packet.thr = 0;
        if (packet.output < 0) packet.output = 0;
        if (packet.rpm < 0) packet.rpm = 0;
        if (packet.voltage_bec < 0) packet.voltage_bec = 0;
        if (packet.current_bec < 0) packet.current_bec = 0;
        if (packet.temperature < 0) packet.temperature = 0;
        handler_(packet);
    }
    index++;
}