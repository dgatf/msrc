#include "functions.h"

uint8_t setCellCount(float voltage)
{
    float level[] = {0, 4.35, 8.7, 13.05, 17.4, 21.75, 26.1, 30.45, 34.8, 34.8, 43.5, 43.5};
    int8_t cont = 11;
    while (voltage < level[cont] && cont > 0)
        cont--;
    return cont + 1;
}

float calcAverage(float alpha, float oldValue, float newValue)
{
    if (isnan(newValue))
        return oldValue;
    return (1 - alpha) * oldValue + alpha * newValue;
}

float Consumption::calcConsumption(float current, uint16_t currentMax)
{
    if (!prevMs)
    {
        prevMs = millis();
        return 0;
    }
    uint16_t now = millis();
    uint16_t interval = (uint16_t)(now - prevMs);
    float mAh = current * interval / 3600.0;
    prevMs = now;
    if (interval > 2000 || (currentMax && (mAh > currentMax * (float)interval / 3600)))
        return 0;
    return mAh;
}

float Vario::calcSpeed(float altitude, uint16_t intervalMin)
{
    uint16_t now = millis();
    uint16_t interval = (uint16_t)(now - prevMs);
    if (interval < intervalMin)
        return speed;
    speed = 1000 * (altitude - prevAltitude) / interval;
    prevAltitude = altitude;
    prevMs = millis();
    return speed;
}

float Vario::calcAltitude(float pressure, float temperature, float P0)
{
    if (P0 == 0)
        return 0;
    return (temperature + 273.15) * (1000 / 6.5) * (1 - pow(pressure / P0, 1 / 5.256));
}