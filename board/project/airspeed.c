#include "airspeed.h"

void airspeed_task(void *parameters)
{
    airspeed_parameters_t parameter = *(airspeed_parameters_t *)parameters;
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    *parameter.airspeed = 0;
    xTaskNotifyGive(receiver_task_handle);
    while (1)
    {
        float pressure = 1000 * (voltage_read(parameter.adc_num) / (TRANSFER_SLOPE * TRANSFER_VCC));
        if (pressure < 0)
            pressure = 0;
        float airspeed = sqrt(2 * pressure / AIR_DENS) / KNOT_TO_MS;
        *parameter.airspeed = get_average(parameter.alpha, *parameter.airspeed, airspeed);
#ifdef SIM_SENSORS
        *parameter.airspeed = 12.34;
#endif
        if (debug)
            printf("\nAirspeed (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.airspeed);
        vTaskDelay(ANALOG_SENSOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}
