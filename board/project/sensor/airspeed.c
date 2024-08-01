#include "airspeed.h"

#include <math.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"

#define AIR_DENS 1.204  // 20ºC, 1atm
#define KNOT_TO_KMH 1.94384
// Transfer function: P(Pa) = 1000 * (Vo/(SLOPE*VCC) - voltageOffset)
// MPXV7002
#define TRANSFER_SLOPE 0.2
#define TRANSFER_VCC 5

void airspeed_task(void *parameters) {
    airspeed_parameters_t parameter = *(airspeed_parameters_t *)parameters;
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    *parameter.airspeed = 0;
    xTaskNotifyGive(context.receiver_task_handle);
    while (1) {
        float pressure = 1000 * (voltage_read(parameter.adc_num) / (TRANSFER_SLOPE * TRANSFER_VCC));
        if (pressure < 0) pressure = 0;
        float airspeed = sqrt(2 * pressure / AIR_DENS) / KNOT_TO_KMH;
        *parameter.airspeed = get_average(parameter.alpha, *parameter.airspeed, airspeed);
#ifdef SIM_SENSORS
        *parameter.airspeed = 12.34;
#endif
        debug("\nAirspeed (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.airspeed);
        vTaskDelay(1000 / parameter.rate / portTICK_PERIOD_MS);
    }
}
