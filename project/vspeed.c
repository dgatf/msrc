#include "vspeed.h"

void vspeed_task(void *parameters)
{
    vspeed_parameters_t *parameter = (vspeed_parameters_t *)parameters;
    *parameter->vspeed = 0;
    vTaskDelay(VSPEED_INIT_DELAY_MS / portTICK_PERIOD_MS);
    float altitude = *parameter->altitude;
    while (1)
    {
        *parameter->vspeed = (*parameter->altitude - altitude) / VSPEED_INTERVAL * 1000;
        altitude = *parameter->altitude;
#ifdef SIM_SENSORS
        *parameter->vspeed = 12.34;
#endif
        if (debug)
            printf("\nVspeed (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter->vspeed);
        vTaskDelay(parameter->interval / portTICK_PERIOD_MS);
    }
}