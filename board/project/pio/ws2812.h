#ifndef WS2812_H
#define WS2812_H

#include "ws2812.pio.h"

void ws2812_init(PIO pio, uint pin, float freq);
void put_pixel_rgb(uint8_t r, uint8_t g, uint8_t b);

#endif