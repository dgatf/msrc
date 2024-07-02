#ifndef PIO_CASTLE
#define PIO_CASTLE

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "castle_link.pio.h"

/*                 castle telemetry

    index   element                          scaler
    0       sync
    1       calib 1 (1000us)                 
    2       Volt (V)                         20
    3       rippleVolt (V)                   4
    4       Curr (A)                         50
    5       Thr (0.1-0.2 ms esc pulse)       1
    6       Output power (%)                 0.2502
    7       rpm                              20416.7
    8       becVolt (V)                      4
    9       becCurr (A)                      4
    10      temp (C) or calib 2 (500us)      30
    11      temp ntc (C) or calib 2 (500us)  63.8125
*/

typedef struct castle_link_telemetry_t {
    float voltage;
    float ripple_voltage;
    float current;
    float thr;
    float output;
    float rpm;
    float voltage_bec;
    float current_bec;
    float temperature;
    bool is_temp_ntc;
} castle_link_telemetry_t;

extern uint8_t debug;

typedef void (*castle_link_handler_t)(castle_link_telemetry_t packet);

void castle_link_init(PIO pio, uint pin_base, uint irq);
void castle_link_set_handler(castle_link_handler_t handler);
void castle_link_remove();

#endif
