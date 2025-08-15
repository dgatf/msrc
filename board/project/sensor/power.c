
#include "power.h"

#include <stdio.h>

#include "auto_offset.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

void power_task(void *parameters) {
    static uint32_t timestamp = 0;
    static uint32_t timestamp_power = 0;
    power_parameters_t parameter = *(power_parameters_t *)parameters;
    *parameter.voltage_current = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.consumption = 0;
    *parameter.power = 0;
    *parameter.power_consumption = 0;
    xTaskNotifyGive(context.receiver_task_handle);
    adc_init();
    adc_gpio_init(parameter.adc_current_num + 26);
    adc_gpio_init(parameter.adc_voltage_num + 26);
    if (parameter.auto_offset) {
        parameter.offset = -1;
        auto_offset_float_parameters_t parameter_auto_offset = {5000, parameter.voltage_current, &parameter.offset};
        TaskHandle_t task_handle;
        xTaskCreate(auto_offset_float_task, "analog_current_auto_offset_task", STACK_AUTO_OFFSET,
                    &parameter_auto_offset, 2, &task_handle);
    }

    while (1) {
        *parameter.voltage_current = voltage_read(parameter.adc_current_num);
        if (parameter.offset != -1)
            *parameter.current = get_average(parameter.alpha, *parameter.current,
                                             (*parameter.voltage_current - parameter.offset) * parameter.multiplier_current);

        *parameter.voltage = get_average(parameter.alpha, *parameter.voltage, voltage_read(parameter.adc_voltage_num) * parameter.multiplier_voltage);
        *parameter.power = *parameter.voltage * *parameter.current;
        if (time_us_32() > 6000000) {
            *parameter.consumption += get_consumption(*parameter.current, 0, &timestamp);
            *parameter.power_consumption += get_energy(*parameter.power, 0, &timestamp_power);
        }

        
#ifdef SIM_SENSORS
        *parameter.voltage = 15.53;       
        *parameter.current = 12.34;
        *parameter.voltage_current = 2.56;
        *parameter.consumption = 120.34;
        *parameter.power = 70.14;
        *parameter.power_consumption = 40;

#endif
        debug("\nCurrent (%u): Curr %.2f Cons %.2f Volt_current %.2f VoltOffs %.2f Multip_curr %.2f",
              uxTaskGetStackHighWaterMark(NULL), *parameter.current, *parameter.consumption, *parameter.voltage_current,
              parameter.offset, parameter.multiplier_current);
        vTaskDelay(1000 / parameter.rate / portTICK_PERIOD_MS);
    }
}
