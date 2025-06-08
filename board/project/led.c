#include "led.h"

#include "pico/stdlib.h"
#include "ws2812.h"

void led_task() {
    const uint WS2812_PIN = 16;
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    ws2812_init(pio1, WS2812_PIN, 800000);

    while (1) {
        for (uint8_t i = context.led_cycles; i > 0; i--) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            put_pixel_rgb(0, 0, 100);
            vTaskDelay(context.led_cycle_duration / 2 / portTICK_PERIOD_MS);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            put_pixel_rgb(0, 0, 0);
            vTaskDelay(context.led_cycle_duration / 2 / portTICK_PERIOD_MS);
        }
        vTaskSuspend(NULL);
    }
}