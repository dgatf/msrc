#include "auto_offset.h"

void auto_offset_task(void *parameters)
{
    auto_offset_parameters_t *parameter = (auto_offset_parameters_t *)parameters;
    vTaskDelay(parameter->delay / portTICK_PERIOD_MS);
    *parameter->offset = *parameter->value;
    if (debug)
    {
        printf("\nAuto offset (%u): Raw offset %i", uxTaskGetStackHighWaterMark(NULL), *parameter->offset);
    }
    vTaskDelete(NULL);
}