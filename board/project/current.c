
#include "current.h"

void current_task(void *parameters)
{
    static uint32_t timestamp = 0;
    current_parameters_t parameter = *(current_parameters_t *)parameters;
    *parameter.current = 0;
    *parameter.consumption = 0;
    xTaskNotifyGive(receiver_task_handle);
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);

    if (parameter.auto_offset)
    {
        auto_offset_parameters_t parameter = {4000, parameter.value, parameter.offset};
        TaskHandle_t task_handle;
        xTaskCreate(auto_offset_task, "auto_offset", STACK_AUTO_OFFSET, NULL, 2, &task_handle);
    }

    while (1)
    {
        *parameter.current = get_average(parameter.alpha, *parameter.current, voltage_read(parameter.adc_num) * parameter.multiplier);
#ifdef SIM_SENSORS
        *parameter.current = 12.34;
        //*parameter.consumption = 120.34;
#endif
        *parameter.current = abs(*parameter.current - parameter.offset);
        if (time_us_32() > 5000000)
        {
            *parameter.consumption += get_consumption(*parameter.current, 0, &timestamp);
        }
        if (debug)
            printf("\nCurrent (%u): Curr %.2f Cons %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.current, *parameter.consumption);
        vTaskDelay(ANALOG_SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

float current_read(uint8_t adc_num)
{
    adc_select_input(adc_num);
    return adc_read() * BOARD_VCC / ADC_RESOLUTION;
}
