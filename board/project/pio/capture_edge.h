#ifndef CAPTURE_EDGE
#define CAPTURE_EDGE

#include "capture_edge.pio.h"

typedef enum edge_type_t { EDGE_NONE, EDGE_FALL, EDGE_RISE } edge_type_t;

typedef void (*capture_handler_t)(uint counter, edge_type_t edge);

uint capture_edge_init(PIO pio, uint pin_base, uint pin_count, float clk_div, uint irq);
void capture_edge_set_handler(uint pin, capture_handler_t handler);

#endif
