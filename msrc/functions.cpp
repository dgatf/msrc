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

float calcConsumption(float current)
{
    static uint32_t lastms = 0;
    if (lastms == 0)
    {
        lastms = millis();
        return 0;
    }
    float mAh= current * (millis() - lastms) / 3600;
    lastms = millis();
    return (mAh);
}
