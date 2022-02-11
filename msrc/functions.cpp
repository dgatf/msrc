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
    static uint16_t lastms = millis();
    uint16_t now = millis();
    uint16_t interval = (uint16_t)(now - lastms);
    float mAh = current * interval / 3600.0;
    lastms = now;
    if (interval > 2000 || (currentMax && (mAh > currentMax * (float)interval / 3600)))
        return 0;
    return mAh;
}

float Vario::calcSpeed(float altitude, uint16_t interval)
{
    static uint16_t ts = 0;
    static float altitudePrev = 0;
    static float speed = 0;
    uint16_t now = millis();
    if ((uint16_t)(now - ts) < interval)
        return speed;
    speed = 1000 * (altitude - altitudePrev) / (uint16_t)(now - ts);
    altitudePrev = altitude;
    ts = millis();
    return speed;
}
