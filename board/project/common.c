
#include "common.h"

#include <math.h>
#include <stdlib.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

float get_average(float alpha, float prev_value, float new_value) {
    if (isnan(new_value)) return prev_value;
    if (alpha > 1)
        return new_value;
    else
        return (1 - alpha) * prev_value + alpha * new_value;
}

float get_consumption(float current, uint16_t current_max, uint32_t *timestamp) {
    if (!*timestamp) {
        *timestamp = time_us_32();
        return 0;
    }
    uint32_t now = time_us_32();                    // us
    uint32_t interval = (now - *timestamp) / 1000;  // ms
    float mah = current * interval / 3600.0;
    *timestamp = now;
    if (interval > 2000 || (current_max && (mah > current_max * interval / 3600.0))) return 0;
    return mah;
}

float voltage_read(uint8_t adc_num) {
    adc_select_input(adc_num);
    return adc_read() * BOARD_VCC / ADC_RESOLUTION;
}

float get_altitude(float pressure, float temperature, float pressure_initial) {
    if (pressure_initial == 0) return 0;
    return (temperature + 273.15) * (1 - pow(pressure / pressure_initial, 1 / 5.256)) / 0.0065;
}
