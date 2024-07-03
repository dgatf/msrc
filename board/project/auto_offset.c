#include "auto_offset.h"

void auto_offset_float_task(void *parameters)
{
    auto_offset_float_parameters_t *parameter = (auto_offset_float_parameters_t *)parameters;
    vTaskDelay(parameter->delay / portTICK_PERIOD_MS);
    *parameter->offset = *parameter->value;
    if (debug)
    {
        printf("\nAuto offset float (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter->offset);
    }
    vTaskDelete(NULL);
}

void auto_offset_int_task(void *parameters)
{
    auto_offset_int_parameters_t *parameter = (auto_offset_int_parameters_t *)parameters;
    vTaskDelay(parameter->delay / portTICK_PERIOD_MS);
    *parameter->offset = *parameter->value;
    if (debug)
    {
        printf("\nAuto offset int (%u): %i", uxTaskGetStackHighWaterMark(NULL), *parameter->offset);
    }
    vTaskDelete(NULL);
}