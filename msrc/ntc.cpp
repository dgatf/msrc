#include "ntc.h"

NtcInterface::NtcInterface(uint8_t pin, uint8_t alpha) : VoltageInterface(pin, alpha) {}

float NtcInterface::read(uint8_t index)
{
    float voltage = readVoltage();
    float ntcR_Rref = (voltage * NTC_R1 / (BOARD_VCC - voltage)) / NTC_R_REF;
    float temperature = 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
    if (temperature < 0)
    {
        temperature = 0;
    }
#ifdef SIM_SENSORS
    return 56;
#endif
    value_ = calcAverage(alpha_, value_, temperature);
    return value_;
}

/* 1 / (NTC_A1 + NTC_B1 * log(ntcR_Rref) + NTC_C1 * pow(log(ntcR_Rref), 2) + NTC_D1 * pow(log(ntcR_Rref), 3)) - 273.15;*/
