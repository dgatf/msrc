#include "cell_count.h"

#include <stdio.h>

#include "pico/stdlib.h"

void cell_count_task(void *parameters) {
    cell_count_parameters_t parameter = *(cell_count_parameters_t *)parameters;
    vTaskDelay(parameter.delay / portTICK_PERIOD_MS);

    float level[] = {0, 4.35, 8.7, 13.05, 17.4, 21.75, 26.1, 30.45, 34.8, 34.8, 43.5, 43.5};
    int cont = 11;

    while (*parameter.voltage < level[cont] && cont > 0) {
        cont--;
    }
    *parameter.cell_count = cont + 1;
    debug("\nCell count (%u): %i", uxTaskGetStackHighWaterMark(NULL), *parameter.cell_count);
    vTaskDelete(NULL);
}