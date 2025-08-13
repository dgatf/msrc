#include "voltage.h"

#include <stdio.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"

void voltage_task(void *parameters) {
    voltage_parameters_t parameter = *(voltage_parameters_t *)parameters;
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    //gpio_pull_down(parameter.adc_num + 26);
    *parameter.voltage = 0;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        *parameter.voltage =
            get_average(parameter.alpha, *parameter.voltage, voltage_read(parameter.adc_num) * parameter.multiplier);
#ifdef SIM_SENSORS
        *parameter.voltage = 12.34;
#endif
        debug("\nVoltage (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.voltage);
        vTaskDelay(1000 / parameter.rate / portTICK_PERIOD_MS);
    }
}
