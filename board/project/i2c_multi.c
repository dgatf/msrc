#include "i2c_multi.h"

static i2c_multi_t *i2c_multi;

static void (*receive_handler)(uint8_t data, bool is_address) = NULL;
static void (*request_handler)(uint8_t address) = NULL;
static void (*stop_handler)(uint8_t length) = NULL;

static inline void start_condition_program_init(PIO pio, uint sm, uint offset, uint pin);
static inline void stop_condition_program_init(PIO pio, uint sm, uint offset, uint pin);
static inline void read_byte_program_init(PIO pio, uint sm, uint offset, uint pin);
static inline void write_byte_program_init(PIO pio, uint sm, uint offset, uint pin);
static inline void byte_handler_pio();
static inline void stop_handler_pio();
static inline uint8_t transpond_byte(uint8_t byte);

void i2c_multi_init(PIO pio, uint pin)
{
    i2c_multi = (i2c_multi_t *)malloc(sizeof(i2c_multi_t));
    i2c_multi->pio = pio;
    i2c_multi->status = I2C_IDLE;
    i2c_multi->pin = pin;
    i2c_multi->bytes_count = 0;
    i2c_multi_disable_all_addresses();
    i2c_multi->buffer = NULL;
    i2c_multi->buffer_start = NULL;
    uint pio_irq0 = (pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0);
    uint pio_irq1 = (pio == pio0 ? PIO0_IRQ_1 : PIO1_IRQ_1);

    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin + 1);

    i2c_multi->offset_start = pio_add_program(pio, &start_condition_program);
    i2c_multi->sm_start = pio_claim_unused_sm(pio, true);
    start_condition_program_init(pio, i2c_multi->sm_start, i2c_multi->offset_start, pin);

    i2c_multi->offset_stop = pio_add_program(pio, &stop_condition_program);
    i2c_multi->sm_stop = pio_claim_unused_sm(pio, true);
    stop_condition_program_init(pio, i2c_multi->sm_stop, i2c_multi->offset_stop, pin);

    i2c_multi->offset_read = pio_add_program(pio, &read_byte_program);
    i2c_multi->sm_read = pio_claim_unused_sm(pio, true);
    read_byte_program_init(pio, i2c_multi->sm_read, i2c_multi->offset_read, pin);

    i2c_multi->offset_write = pio_add_program(pio, &write_byte_program);
    i2c_multi->sm_write = pio_claim_unused_sm(pio, true);
    write_byte_program_init(pio, i2c_multi->sm_write, i2c_multi->offset_write, pin);

    pio_sm_put_blocking(pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[1]) << 16) | do_ack_program_instructions[0]);
    pio_sm_put_blocking(pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[3]) << 16) | do_ack_program_instructions[2]);

    irq_set_exclusive_handler(pio_irq0, byte_handler_pio);
    irq_set_enabled(pio_irq0, true);
    irq_set_exclusive_handler(pio_irq1, stop_handler_pio);
    irq_set_enabled(pio_irq1, true);
}

void i2c_multi_set_write_buffer(uint8_t *buffer)
{
    i2c_multi->buffer = buffer;
    i2c_multi->buffer_start = buffer;
}

void i2c_multi_set_receive_handler(i2c_multi_receive_handler_t handler)
{
    receive_handler = handler;
}

void i2c_multi_set_request_handler(i2c_multi_request_handler_t handler)
{
    request_handler = handler;
}

void i2c_multi_set_stop_handler(i2c_multi_stop_handler_t handler)
{
    stop_handler = handler;
}

void i2c_multi_enable_address(uint8_t address)
{
    i2c_multi->address[address / 32] |= 1 << (address % 32);
}

void i2c_multi_disable_address(uint8_t address)
{
    i2c_multi->address[address / 32] &= ~(1 << (address % 32));
}

void i2c_multi_enable_all_addresses()
{
    i2c_multi->address[0] = 0xFFFFFFFF;
    i2c_multi->address[1] = 0xFFFFFFFF;
    i2c_multi->address[2] = 0xFFFFFFFF;
    i2c_multi->address[3] = 0xFFFFFFFF;
}

void i2c_multi_disable_all_addresses()
{
    i2c_multi->address[0] = 0;
    i2c_multi->address[1] = 0;
    i2c_multi->address[2] = 0;
    i2c_multi->address[3] = 0;
}

bool i2c_multi_is_address_enabled(uint8_t address)
{
    return i2c_multi->address[address / 32] & (1 << (address % 32));
}

void i2c_multi_disable()
{
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_read, false);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_write, false);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_start, false);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_stop, false);
    pio_sm_clear_fifos(i2c_multi->pio, i2c_multi->sm_read);
    pio_sm_clear_fifos(i2c_multi->pio, i2c_multi->sm_write);
    gpio_set_input_enabled(i2c_multi->pin, true);
    gpio_set_input_enabled(i2c_multi->pin + 1, true);
    i2c_multi->bytes_count = 0;
    i2c_multi->status = I2C_IDLE;
    i2c_multi->buffer = i2c_multi->buffer_start;
}

void i2c_multi_restart()
{
    i2c_multi_disable();
    pio_sm_restart(i2c_multi->pio, i2c_multi->sm_start);
    pio_sm_restart(i2c_multi->pio, i2c_multi->sm_stop);
    pio_sm_restart(i2c_multi->pio, i2c_multi->sm_read);
    pio_sm_restart(i2c_multi->pio, i2c_multi->sm_write);
    pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[1]) << 16) | do_ack_program_instructions[0]);
    pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[3]) << 16) | do_ack_program_instructions[2]);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_read, true);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_write, true);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_start, true);
    pio_sm_set_enabled(i2c_multi->pio, i2c_multi->sm_stop, true);
}

void i2c_multi_remove()
{
    receive_handler = NULL;
    request_handler = NULL;
    stop_handler = NULL;
    pio_set_irq0_source_enabled(i2c_multi->pio, pis_interrupt0, false);
    pio_set_irq1_source_enabled(i2c_multi->pio, pis_interrupt1, false);
    pio_clear_instruction_memory(i2c_multi->pio);
    pio_sm_unclaim(i2c_multi->pio, i2c_multi->sm_start);
    pio_sm_unclaim(i2c_multi->pio, i2c_multi->sm_stop);
    pio_sm_unclaim(i2c_multi->pio, i2c_multi->sm_read);
    pio_sm_unclaim(i2c_multi->pio, i2c_multi->sm_write);
    i2c_multi->buffer = NULL;
    i2c_multi->buffer_start = NULL;
    i2c_multi->bytes_count = 0;
    i2c_multi->status = I2C_IDLE;
    gpio_set_input_enabled(i2c_multi->pin, true);
    gpio_set_input_enabled(i2c_multi->pin + 1, true);
    free(i2c_multi);
}

static inline void start_condition_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_sm_config c = start_condition_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_clkdiv(&c, 10);
    sm_config_set_jmp_pin(&c, pin + 1);
    pio_sm_init(pio, sm, offset + start_condition_offset_start, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void stop_condition_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_sm_config c = stop_condition_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_clkdiv(&c, 10);
    sm_config_set_jmp_pin(&c, pin + 1);
    pio_sm_init(pio, sm, offset + stop_condition_offset_start, &c);
    pio_sm_set_enabled(pio, sm, true);
    pio_set_irq1_source_enabled(pio, pis_interrupt1, true);
    pio_interrupt_clear(pio, 1);
}

static inline void read_byte_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_sm_config c = read_byte_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_clkdiv(&c, 10);
    sm_config_set_out_shift(&c, true, true, 32);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    pio_interrupt_clear(pio, 0);
    sm_config_set_set_pins(&c, pin, 2);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void write_byte_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_sm_config c = write_byte_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_set_pins(&c, pin, 2);
    sm_config_set_clkdiv(&c, 10);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_jmp_pin(&c, pin);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void byte_handler_pio()
{
    uint8_t received = 0;
    bool is_address = false;
    i2c_multi->bytes_count++;
    if (i2c_multi->status != I2C_WRITE)
    {
        received = transpond_byte(pio_sm_get_blocking(i2c_multi->pio, i2c_multi->sm_read) >> 24); // Do the bit-reverse here as PIO instructions are scarce
    }
    if (i2c_multi->status == I2C_IDLE)
    {
        if (!i2c_multi_is_address_enabled(received >> 1))
        {
            i2c_multi->status = I2C_IDLE;
            i2c_multi->bytes_count = 0;
            pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[10] + i2c_multi->offset_read) << 16) | do_ack_program_instructions[9]);
            pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[1]) << 16) | do_ack_program_instructions[0]);
            pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[3]) << 16) | do_ack_program_instructions[2]);
            pio_interrupt_clear(i2c_multi->pio, 0);
            return;
        }
        if (received & 1)
        {
            i2c_multi->status = I2C_WRITE;
        }
        else
        {
            i2c_multi->status = I2C_READ;
        }
        is_address = true;
    }
    if (i2c_multi->status == I2C_READ)
    {
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[5]) << 16) | do_ack_program_instructions[4]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[7] + i2c_multi->offset_read) << 16) | do_ack_program_instructions[6]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[1]) << 16) | do_ack_program_instructions[0]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[3]) << 16) | do_ack_program_instructions[2]);
        if (receive_handler)
        {
            if (is_address)
                receive_handler(received >> 1, true);
            else
                receive_handler(received, false);
        }
    }
    if (i2c_multi->status == I2C_WRITE && is_address)
    {
        if (request_handler)
        {
            request_handler(received >> 1);
        }
        uint8_t value = 0;
        if (i2c_multi->buffer)
        {
            value = transpond_byte(*i2c_multi->buffer);
            i2c_multi->buffer++;
        }
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[5]) << 16) | do_ack_program_instructions[4]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[8] + i2c_multi->offset_read) << 16) | do_ack_program_instructions[6]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[1]) << 16) | do_ack_program_instructions[0]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_read, (((uint32_t)do_ack_program_instructions[3]) << 16) | do_ack_program_instructions[2]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, value);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[1]) << 16) | wait_ack_program_instructions[0]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[3]) << 16) | wait_ack_program_instructions[2]);
    }
    if (i2c_multi->status == I2C_WRITE && !is_address)
    {
        uint8_t value = 0;
        if (i2c_multi->buffer)
        {
            value = transpond_byte(*i2c_multi->buffer);
            i2c_multi->buffer++;
        }
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[5]) << 16) | wait_ack_program_instructions[4]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[7] + i2c_multi->offset_write) << 16) | (wait_ack_program_instructions[6]));
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[9] + i2c_multi->offset_write) << 16) | (wait_ack_program_instructions[8]));
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, value);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[1]) << 16) | wait_ack_program_instructions[0]);
        pio_sm_put_blocking(i2c_multi->pio, i2c_multi->sm_write, (((uint32_t)wait_ack_program_instructions[3]) << 16) | wait_ack_program_instructions[2]);
    }
    pio_interrupt_clear(i2c_multi->pio, 0);
}

static inline void stop_handler_pio()
{
    pio_interrupt_clear(i2c_multi->pio, 1);
    if (i2c_multi->status == I2C_IDLE)
        return;
    pio_sm_exec(i2c_multi->pio, i2c_multi->sm_read, i2c_multi->offset_read);
    pio_sm_clear_fifos(i2c_multi->pio, i2c_multi->sm_write);
    pio_sm_exec(i2c_multi->pio, i2c_multi->sm_write, wait_ack_program_instructions[11]);
    pio_sm_exec(i2c_multi->pio, i2c_multi->sm_write, i2c_multi->offset_write);
    i2c_multi->buffer = i2c_multi->buffer_start;
    if (stop_handler)
    {
        stop_handler(i2c_multi->bytes_count - 1);
    }
    i2c_multi->bytes_count = 0;
    i2c_multi->status = I2C_IDLE;
}

static inline uint8_t transpond_byte(uint8_t byte)
{
    uint8_t transponded = ((byte & 0x1) << 7) |
                          (((byte & 0x2) >> 1) << 6) |
                          (((byte & 0x4) >> 2) << 5) |
                          (((byte & 0x8) >> 3) << 4) |
                          (((byte & 0x10) >> 4) << 3) |
                          (((byte & 0x20) >> 5) << 2) |
                          (((byte & 0x40) >> 6) << 1) |
                          (((byte & 0x80) >> 7));
    return transponded;
}
