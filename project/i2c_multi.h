#ifndef I2C_MULTI
#define I2C_MULTI

#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "i2c_multi.pio.h"

typedef enum i2c_multi_status_t
{
    I2C_IDLE,
    I2C_READ,
    I2C_WRITE
} i2c_multi_status_t;

typedef void (*i2c_multi_receive_handler_t)(uint8_t data, bool is_address);
typedef void (*i2c_multi_request_handler_t)(uint8_t address);
typedef void (*i2c_multi_stop_handler_t)(uint8_t length);

typedef struct i2c_multi_t
{
    PIO pio;
    uint offset_read, offset_write, sm_read, sm_write, offset_start, offset_stop, sm_start, sm_stop, pin;
    i2c_multi_status_t status;
    uint8_t *buffer, *buffer_start;
    uint8_t bytes_count;
    uint address[4];
} i2c_multi_t;

void i2c_multi_init(PIO pio, uint pin);
void i2c_multi_set_write_buffer(uint8_t *buffer);
void i2c_multi_set_receive_handler(i2c_multi_receive_handler_t handler);
void i2c_multi_set_request_handler(i2c_multi_request_handler_t handler);
void i2c_multi_set_stop_handler(i2c_multi_stop_handler_t handler);
void i2c_multi_enable_address(uint8_t address);
void i2c_multi_disable_address(uint8_t address);
void i2c_multi_enable_all_addresses();
void i2c_multi_disable_all_addresses();
bool i2c_multi_is_address_enabled(uint8_t address);
void i2c_multi_disable();
void i2c_multi_restart();
void i2c_multi_remove();

#endif
