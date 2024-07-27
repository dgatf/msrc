#include "ntc.h"

void ntc_task(void *parameters)
{
    ntc_parameters_t parameter = *(ntc_parameters_t *)parameters;
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    *parameter.ntc = 0;
    xTaskNotifyGive(receiver_task_handle);
    while (1)
    {
        float voltage = voltage_read(parameter.adc_num);
        float ntcR_Rref = (voltage * NTC_R1 / (BOARD_VCC - voltage)) / NTC_R_REF;
        if (ntcR_Rref < 1)
            ntcR_Rref = 1;
        float temperature = 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
        *parameter.ntc = get_average(parameter.alpha, *parameter.ntc, temperature);
#ifdef SIM_SENSORS
        *parameter.ntc = 12.34;
#endif
        if (debug)
            printf("\nTemperature (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.ntc);
        vTaskDelay(1000 / parameter.rate  / portTICK_PERIOD_MS);
    }
}