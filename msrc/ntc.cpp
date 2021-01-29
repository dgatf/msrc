#include "ntc.h"

Ntc::Ntc(uint8_t pin, uint8_t alpha) : Voltage(pin, alpha) {}

void Ntc::update()
{
    float voltage = readVoltage();
    float ntcR_Rref = (voltage * NTC_R1 / (BOARD_VCC - voltage)) / NTC_R_REF;
    float temperature = 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
    if (temperature < 0)
    {
        temperature = 0;
    }
    value_ = calcAverage(alpha_ / 100.0F, value_, temperature);
#ifdef SIM_SENSORS
    value_ = 12.34;
#endif
}

/* 1 / (NTC_A1 + NTC_B1 * log(ntcR_Rref) + NTC_C1 * pow(log(ntcR_Rref), 2) + NTC_D1 * pow(log(ntcR_Rref), 3)) - 273.15;*/
