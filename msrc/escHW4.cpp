#include "escHW4.h"

EscHW4::EscHW4(Stream &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp, uint8_t type) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp), type_(type) {}

void EscHW4::update()
{
    while (serial_.available() >= 13)
    {
        uint8_t cont;
        if (serial_.read() == 0x9B)
        {
            uint8_t data[18];
            if (serial_.peek() == 0x9B) // esc signature
            {
                cont = serial_.readBytes(data, 12);
                //if (type_ == ESCHW4_TYPE_V5_HV + 1)
                //{
                //    uint8_t i = 0;
                /*while (memcmp(data, signature_[i], 12) != 0 && i < 4)
                    {
                        i++;
                    }
                    if (memcmp(data, signature_[i], 12) == 0)
                    {
                        type_ = i;
                    }*/
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
                DEBUG_SERIAL.print("S:");
                for (int i = 0; i < 12; i++)
                {
                    DEBUG_SERIAL.print(data[i], HEX);
                    DEBUG_SERIAL.print(" ");
                }
                DEBUG_SERIAL.println();
#endif
                //}
            }
            else
            {
                cont = serial_.readBytes(data, 18);
                if (cont == 18)
                {
                    thr_ = (uint16_t)data[3] << 8 | data[4]; // 0-1024
                    pwm_ = (uint16_t)data[5] << 8 | data[6]; // 0-1024
                    float rpm = (uint32_t)data[7] << 16 | (uint16_t)data[8] << 8 | data[9];
                    float voltage = calcVolt((uint16_t)data[10] << 8 | data[11]);
                    float current = calcCurr((uint16_t)data[12] << 8 | data[13]);
                    float tempFET = calcTemp((uint16_t)data[14] << 8 | data[15]);
                    float tempBEC = calcTemp((uint16_t)data[16] << 8 | data[17]);
                    rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
                    voltage_ = calcAverage(alphaVolt_ / 100.0F, voltage_, voltage);
                    current_ = calcAverage(alphaCurr_ / 100.0F, current_, current);
                    tempFet_ = calcAverage(alphaTemp_ / 100.0F, tempFet_, tempFET);
                    tempBec_ = calcAverage(alphaTemp_ / 100.0F, tempBec_, tempBEC);
                    if (cellCount_ == 255)
                        if (millis() > 10000 && voltage_ > 1)
                            cellCount_ = setCellCount(voltage_);
                    cellVoltage_ = voltage_ / cellCount_;
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
                    uint32_t pn =
                        (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
                    DEBUG_SERIAL.print("N:");
                    DEBUG_SERIAL.print(pn);
                    DEBUG_SERIAL.print(" R:");
                    DEBUG_SERIAL.print(rpm);
                    DEBUG_SERIAL.print(" V:");
                    DEBUG_SERIAL.print(voltage);
                    DEBUG_SERIAL.print(" C:");
                    DEBUG_SERIAL.print(current);
                    DEBUG_SERIAL.print(" TF:");
                    DEBUG_SERIAL.print(tempFET);
                    DEBUG_SERIAL.print(" TB:");
                    DEBUG_SERIAL.print(tempBEC);
                    DEBUG_SERIAL.print(" VC:");
                    DEBUG_SERIAL.println(cellVoltage_);
#endif
                }
            }
        }
    }
#ifdef SIM_SENSORS
    rpm_ = 12345.67;
    voltage_ = 12.34;
    current_ = 5.678;
    tempFet_ = 12.34;
    tempBec_ = 12.34;
    cellVoltage_ = voltage_ / cellCount_;
#endif
}

float EscHW4::calcVolt(uint16_t voltRaw)
{
    return ESCHW4_V_REF * voltRaw / ESCHW4_ADC_RES * voltageDivisor_[type_];
}

float EscHW4::calcTemp(uint16_t tempRaw)
{
    float voltage = tempRaw * ESCHW4_V_REF / ESCHW4_ADC_RES;
    float ntcR_Rref = (voltage * ESCHW4_NTC_R1 / (ESCHW4_V_REF - voltage)) / ESCHW4_NTC_R_REF;
    float temperature = 1 / (log(ntcR_Rref) / ESCHW4_NTC_BETA + 1 / 298.15) - 273.15;
    if (temperature < 0)
        return 0;
    return temperature;
}

float EscHW4::calcCurr(uint16_t currentRaw)
{
    if (thr_ < 128 || currentRaw - rawCurrentOffset_[type_] < 0)
        return 0;
    return (currentRaw - rawCurrentOffset_[type_]) * ESCHW4_V_REF / (ESCHW4_DIFFAMP_GAIN * ESCHW4_DIFFAMP_SHUNT * ESCHW4_ADC_RES);
}

uint16_t *EscHW4::thrP()
{
    return &thr_;
}

uint16_t *EscHW4::pwmP()
{
    return &pwm_;
}

float *EscHW4::rpmP()
{
    return &rpm_;
}

float *EscHW4::voltageP()
{
    return &voltage_;
}

float *EscHW4::currentP()
{
    return &current_;
}

float *EscHW4::tempFetP()
{
    return &tempFet_;
}

float *EscHW4::tempBecP()
{
    return &tempBec_;
}
