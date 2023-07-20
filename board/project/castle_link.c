#include "castle_link.h"

static uint sm_pulse_, sm_counter_, offset_pulse_, offset_counter_;
static PIO pio_;
static void (*handler_)(castle_link_telemetry_t packet) = {NULL};

static inline void handler_pio();

void castle_link_init(PIO pio, uint pin, uint irq)
{
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

void castle_link_set_handler(castle_link_handler_t handler)
{
    handler_ = handler;
}

void castle_link_remove()
{
    castle_link_set_handler(NULL);
    pio_remove_program(pio_, &pulse_program, offset_pulse_);
    pio_remove_program(pio_, &counter_program, offset_counter_);
    pio_sm_unclaim(pio_, sm_pulse_);
    pio_sm_unclaim(pio_, sm_counter_);
}

static inline void handler_pio()
{
    static uint index = 0;
    static const float scaler[11] = {0, 20, 4, 50, 1, 0.2502, 20416.7, 4, 4, 30, 63.8125};
    static uint value[12];

    if (pio_sm_is_rx_fifo_full(pio_, sm_counter_))
    {
        pio_sm_clear_fifos(pio_, sm_counter_);
        return;
    }
    /*bool sync = pio_sm_get_blocking(pio_, sm_counter_);
    if (sync)
    {
        index = 0;
        printf("\n");
        return;
    }*/
    uint data = pio_sm_get_blocking(pio_, sm_counter_);
    if (data > 50000)
    {
        index = 0;
        //printf("%i \n", data);
        return;
    }
    value[index] = data;
    //printf("(%u)%u ", index, value[index]);
    
    if (index == 10)
    {
        uint calibration;
        castle_link_telemetry_t packet;
        if (value[9] < value[10])
        {
            calibration = value[0] / 2 + value[9];
            packet.is_temp_ntc = true;
            float temp_raw = value[10] * scaler[10] / calibration;
            packet.temperature = 1 / (log(temp_raw * 10200 / (255 - temp_raw) / 10000) / 3455 + 1 / 298) - 273;
        }
        else
        {
            calibration = value[0] / 2 + value[10];
            packet.is_temp_ntc = false;
            packet.temperature = value[9] * scaler[9] / calibration;
        }
        packet.voltage = (value[1] - calibration / 2) * scaler[1] / calibration;
        packet.ripple_voltage = (value[2] - calibration / 2) * scaler[2] / calibration;
        packet.current = (value[3] - calibration / 2) * scaler[3] / calibration;
        packet.thr = (value[4] - calibration / 2) * scaler[4] / calibration;
        packet.output = (value[5] - calibration / 2) * scaler[5] / calibration;
        packet.rpm = (value[6] - calibration / 2) * scaler[6] / calibration;
        packet.voltage_bec = (value[7] - calibration / 2) * scaler[7] / calibration;
        packet.current_bec = (value[8] - calibration / 2) * scaler[8] / calibration;
        handler_(packet);
    }
    index++;
    pio_interrupt_clear(pio_, CASTLE_LINK_IRQ_NUM);
}
