#include "gpio.h"

#include <stdio.h>

#include "pico/stdlib.h"

#define GPIO_MASK (1 << 17)

void gpio_task(void *parameters) {
    gpio_parameters_t parameter = *(gpio_parameters_t *)parameters;
    gpio_init_mask(GPIO_MASK);
    gpio_set_dir_in_masked(GPIO_MASK);
    xTaskNotifyGive(context.receiver_task_handle);

    while (1) {
        uint8_t value = 0;
        for (uint i = 0; i < 6; i++) {
            if (parameter.mask & (1 << i)) value |= gpio_get(i + 17) << i;
        }
#ifdef SIM_SENSORS
        value = 0B101;
#endif
        *parameter.value = value;

        debug("\nGPIO (%u): 0x%X", uxTaskGetStackHighWaterMark(NULL), value);

        vTaskDelay(parameter.interval / portTICK_PERIOD_MS);
    }
}
