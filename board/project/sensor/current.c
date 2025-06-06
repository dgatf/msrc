
#include "current.h"

#include <stdio.h>

#include "auto_offset.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

void current_task(void *parameters) {
    static uint32_t timestamp = 0;
    current_parameters_t parameter = *(current_parameters_t *)parameters;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.consumption = 0;
    xTaskNotifyGive(context.receiver_task_handle);
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    gpio_pull_down(parameter.adc_num + 26);
    if (parameter.auto_offset) {
        parameter.offset = -1;
        auto_offset_float_parameters_t parameter_auto_offset = {5000, parameter.voltage, &parameter.offset};
        TaskHandle_t task_handle;
        xTaskCreate(auto_offset_float_task, "analog_current_auto_offset_task", STACK_AUTO_OFFSET,
                    &parameter_auto_offset, 2, &task_handle);
    }

    while (1) {
        *parameter.voltage = voltage_read(parameter.adc_num);
        if (parameter.offset != -1)
            *parameter.current = get_average(parameter.alpha, *parameter.current,
                                             (*parameter.voltage - parameter.offset) * parameter.multiplier);

        if (time_us_32() > 6000000) {
            *parameter.consumption += get_consumption(*parameter.current, 0, &timestamp);
        }
#ifdef SIM_SENSORS
        *parameter.current = 12.34;
        *parameter.consumption = 120.34;
#endif
        debug("\nCurrent (%u): Curr %.2f Cons %.2f Volt %.2f VoltOffs %.2f Multip %.2f",
              uxTaskGetStackHighWaterMark(NULL), *parameter.current, *parameter.consumption, *parameter.voltage,
              parameter.offset, parameter.multiplier);
        vTaskDelay(1000 / parameter.rate / portTICK_PERIOD_MS);
    }
}

float current_read(uint8_t adc_num) {
    adc_select_input(adc_num);
    return adc_read() * BOARD_VCC / ADC_RESOLUTION;
}
