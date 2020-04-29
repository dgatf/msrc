#include "escHW4.h"

EscHW4Interface::EscHW4Interface(Stream &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp) {}

bool EscHW4Interface::update()
{
    while (serial_.available() >= 13)
    {
        uint8_t header = serial_.read();
        if (header == 0x9B)
        {
            if (serial_.peek() == 0x9B)
            {
                return false;
            }
            uint8_t data[18];
            uint8_t cont = serial_.readBytes(data, 18);
            if (cont == 18 && data[0] != 0x9B)
            {
                uint16_t thr = (uint16_t)data[3] << 8 | data[4]; // 0-1024
                uint16_t pwm = (uint16_t)data[5] << 8 | data[6]; // 0-1024
                float rpm = (uint32_t)data[7] << 16 | (uint16_t)data[8] << 8 | data[9];
                float voltage = (float)((uint16_t)data[10] << 8 | data[11]) / 113;
                float current = calcCurrHW((uint16_t)data[12] << 8 | data[13]);
                float tempFET = calcTempHW((uint16_t)data[14] << 8 | data[15]);
                float tempBEC = calcTempHW((uint16_t)data[16] << 8 | data[17]);
                value_[ESCHW4_RPM] = calcAverage(alphaRpm_, rpm, value_[ESCHW4_RPM]);
                value_[ESCHW4_VOLTAGE] = calcAverage(alphaVolt_ / 100.0F, voltage, value_[ESCHW4_VOLTAGE]);
                value_[ESCHW4_CURRENT] = calcAverage(alphaCurr_ / 100.0F, current, value_[ESCHW4_CURRENT]);
                value_[ESCHW4_TEMPFET] = calcAverage(alphaTemp_ / 100.0F, tempFET, value_[ESCHW4_TEMPFET]);
                value_[ESCHW4_TEMPBEC] = calcAverage(alphaTemp_ / 100.0F, tempBEC, value_[ESCHW4_TEMPBEC]);
                value_[ESCHW4_CELL_VOLTAGE] = value_[ESCHW4_VOLTAGE] / cellCount_;
#ifdef DEBUG
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

float EscHW4Interface::calcTempHW(uint16_t tempRaw)
{
    uint16_t tempFunc[26][2] =
        {{0, 1},
         {14, 2},
         {28, 3},
         {58, 5},
         {106, 8},
         {158, 11},
         {234, 15},
         {296, 18},
         {362, 21},
         {408, 23},
         {505, 27},
         {583, 30},
         {664, 33},
         {720, 35},
         {807, 38},
         {897, 41},
         {1021, 45},
         {1150, 49},
         {1315, 54},
         {1855, 70},
         {1978, 74},
         {2239, 82},
         {2387, 87},
         {2472, 90},
         {2656, 97},
         {2705, 99}};
    if (tempRaw > 3828)
        return 0;
    if (tempRaw < 1123)
        return 100;
    tempRaw = 3828 - tempRaw;
    uint8_t i = 0;
    while (i < 26 && tempRaw >= tempFunc[i][0])
    {
        i++;
    }
    return tempFunc[i - 1][1] + (tempFunc[i][1] - tempFunc[i - 1][1]) * (float)(tempRaw - tempFunc[i - 1][0]) / (tempFunc[i][0] - tempFunc[i - 1][0]);
}

float EscHW4Interface::calcCurrHW(uint16_t currentRaw)
{
    if (currentRaw > 28)
    {
        return (float)(currentRaw - 28) / 610;
    }
    else
        return 0;
}

float EscHW4Interface::read(uint8_t index)
{
#ifdef SIM_SENSORS
    value_[ESCHW4_RPM] = 10000;
    value_[ESCHW4_VOLTAGE] = 24;
    value_[ESCHW4_CURRENT] = 2;
    value_[ESCHW4_TEMPFET] = 50;
    value_[ESCHW4_TEMPBEC] = 30;
    value_[ESCHW4_CELL_VOLTAGE] = value_[ESCHW4_VOLTAGE] / cellCount_;
    if (index >= 0 && index < 6)
    {
        update();
        if (index == ESCHW4_CELL_VOLTAGE && cellCount_ == 0xFF) {
            if (millis() > 10000)
            {
                cellCount_ = setCellCount(value_[ESCHW4_VOLTAGE]);
            }  
        }
        return value_[index];
    }
    return 0;
#endif
    if (index >= 0 && index < 5)
    {
        update();
        return value_[index];
    }
    return 0;
}