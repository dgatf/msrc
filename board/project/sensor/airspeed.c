#include "airspeed.h"

#include <math.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"

/* If there is not barometer sensor installed, values used to calculate air density are:
   Nominal pressure at sea level 101325 hPa.
   Defalut temperature value 20ºC
*/

#define KNOT_TO_KMH 1.94384
#define AIR_CONSTANT_R 287.05  // J/(kg.K)

void airspeed_task(void *parameters) {
    airspeed_parameters_t parameter = *(airspeed_parameters_t *)parameters;
    adc_init();
    adc_gpio_init(parameter.adc_num + 26);
    gpio_pull_down(parameter.adc_num + 26);
    *parameter.airspeed = 0;
    xTaskNotifyGive(context.receiver_task_handle);
    float temperature, pressure, delta_pressure, air_density, airspeed;
    static float voltage = 0;

    while (1) {
        if (!parameter.temperature)
            temperature = 20;
        else
            temperature = *parameter.temperature;  // ºC
        if (!parameter.pressure)
            pressure = 101325;
        else
            pressure = *parameter.pressure;  // Pa
        voltage = voltage_read(parameter.adc_num);
        //voltage += 0.1;
        //if (voltage > 3.3) voltage = 0;
        air_density = pressure / (AIR_CONSTANT_R * (temperature + 273.15));
        delta_pressure = ((voltage - parameter.offset) / parameter.slope - 2.5) * 1000;  // Pa
        // Formula: speed = sqrt(2*P/air_dens) -> Units: P (Pa=N/m2), air_dens (kg/m3), N (kg*m/s2) -> speed =
        // sqrt(kg*m/s2/m2*m3/kg) = sqrt(m2/s2) = m/s
        if (delta_pressure < 0)
            airspeed = /*-1 **/ sqrt(-2 * delta_pressure / air_density);
        else
            airspeed = sqrt(2 * delta_pressure / air_density);
        airspeed *= 3.6; // m/s to km/h
        *parameter.airspeed = get_average(parameter.alpha, *parameter.airspeed, airspeed);
#ifdef SIM_SENSORS
        *parameter.airspeed = 123.34;
#endif
        debug("\nAirspeed (%u): %.2f (P(%u)%.2f T(%u)%.2f V %.2f AD %.2f dP %.2f)", uxTaskGetStackHighWaterMark(NULL),
              *parameter.airspeed, parameter.pressure, pressure, parameter.temperature, temperature, voltage,
              air_density, delta_pressure);

        vTaskDelay(1000 / parameter.rate / portTICK_PERIOD_MS);
    }
}
