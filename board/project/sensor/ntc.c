#include "ntc.h"

#include <math.h>
#include <stdio.h>

#include "hardware/adc.h"

// Thermistors (NTC 100k, R1 10k)
#define NTC_R_REF 100000UL
#define NTC_R1 10000
#define NTC_BETA 4190
// #define NTC_A1 3.35E-03
// #define NTC_B1 2.46E-04
// #define NTC_C1 3.41E-06
// #define NTC_D1 1.03E-07

void ntc_task(void *parameters) {
    ntc_parameters_t parameter = *(ntc_parameters_t *)parameters;
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    //gpio_pull_down(parameter.adc_num + 26);
    *parameter.ntc = 0;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        float voltage = voltage_read(parameter.adc_num);
        float ntcR_Rref = (voltage * NTC_R1 / (BOARD_VCC - voltage)) / NTC_R_REF;
        if (ntcR_Rref < 0.0001) ntcR_Rref = 0.0001;
        float temperature = 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
        *parameter.ntc = get_average(parameter.alpha, *parameter.ntc, temperature);
#ifdef SIM_SENSORS
        *parameter.ntc = 12.34;
#endif
        debug("\nTemperature (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.ntc);
        vTaskDelay(1000 / parameter.rate / portTICK_PERIOD_MS);
    }
}