#include "escHW4.h"

EscHW4::EscHW4(HardwareSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp, uint8_t type) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp), type_(type) {}

void EscHW4::begin()
{
    serial_.begin(19200);
    serial_.setTimeout(ESCHW4_ESCSERIAL_TIMEOUT);
}

bool EscHW4::update()
{
    while (serial_.available() >= 13)
    {
        uint8_t header = serial_.read();
        uint8_t cont;
        if (header == 0x9B)
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
#ifdef DEBUG_SIGNATURE
                    Serial.print("ESC TYPE [");
                    Serial.print(type_);
                    Serial.print("]: ");
                    for (int i = 0; i < 12; i++)
                    {
                        Serial.print(data[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
#endif
                //}
                return false;
            }
            cont = serial_.readBytes(data, 18);
            if (cont == 18)
            {
                value_[ESCHW4_THR] = (uint16_t)data[3] << 8 | data[4]; // 0-1024
                value_[ESCHW4_PWM] = (uint16_t)data[5] << 8 | data[6]; // 0-1024
                float rpm = (uint32_t)data[7] << 16 | (uint16_t)data[8] << 8 | data[9];
                float voltage = calcVolt((uint16_t)data[10] << 8 | data[11]);
                float current = calcCurr((uint16_t)data[12] << 8 | data[13]);
                float tempFET = calcTemp((uint16_t)data[14] << 8 | data[15]);
                float tempBEC = calcTemp((uint16_t)data[16] << 8 | data[17]);
                value_[ESCHW4_RPM] = calcAverage(alphaRpm_  / 100.0F, value_[ESCHW4_RPM], rpm);
                value_[ESCHW4_VOLTAGE] = calcAverage(alphaVolt_ / 100.0F, value_[ESCHW4_VOLTAGE], voltage);
                value_[ESCHW4_CURRENT] = calcAverage(alphaCurr_ / 100.0F, value_[ESCHW4_CURRENT], current);
                value_[ESCHW4_TEMPFET] = calcAverage(alphaTemp_ / 100.0F, value_[ESCHW4_TEMPFET], tempFET);
                value_[ESCHW4_TEMPBEC] = calcAverage(alphaTemp_ / 100.0F, value_[ESCHW4_TEMPBEC], tempBEC);
                value_[ESCHW4_CELL_VOLTAGE] = value_[ESCHW4_VOLTAGE] / cellCount_;
#ifdef DEBUG_ESC
                uint32_t pn =
                    (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
                serial_.print("PN: ");
                serial_.print(pn);
                serial_.print(" RPM: ");
                serial_.print(rpm);
                serial_.print(" Volt: ");
                serial_.print(voltage);
                serial_.print(" Curr: ");
                serial_.print(current);
                serial_.print(" tempFET: ");
                serial_.print(tempFET);
                serial_.print(" tempBEC: ");
                serial_.print(tempBEC);
                /*serial_.print(" ");
                //for (uint8_t i = 0; i < cont; i++) {
                serial_.print(data[i], HEX);
                serial_.print(" ");
                }*/
                serial_.println();
#endif
                return true;
            }
        }
    }
    return false;
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
    if (value_[ESCHW4_THR] < 128 || currentRaw - rawCurrentOffset_[type_] < 0)
        return 0;
    return (currentRaw - rawCurrentOffset_[type_]) * ESCHW4_V_REF / (ESCHW4_DIFFAMP_GAIN * ESCHW4_DIFFAMP_SHUNT * ESCHW4_ADC_RES);
}

float EscHW4::read(uint8_t index)
{
    if (index <= ESCHW4_CELL_VOLTAGE)
    {
#ifdef SIM_SENSORS
        value_[ESCHW4_RPM] = 10000;
        value_[ESCHW4_VOLTAGE] = 24;
        value_[ESCHW4_CURRENT] = 2;
        value_[ESCHW4_TEMPFET] = 50;
        value_[ESCHW4_TEMPBEC] = 30;
        value_[ESCHW4_CELL_VOLTAGE] = value_[ESCHW4_VOLTAGE] / cellCount_;
#else
        update();
#endif
        if (index == ESCHW4_CELL_VOLTAGE && cellCount_ == 0xFF)
        {
            if (millis() > 10000)
            {
                cellCount_ = setCellCount(value_[ESCHW4_VOLTAGE]);
            }
        }
        return value_[index];
    }
    return 0;
}