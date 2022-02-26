#include "escApdF.h"

EscApdF::EscApdF(AbstractSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp)
{
}

void EscApdF::begin()
{
    serial_.begin(115200, SERIAL__8N1);
    serial_.setTimeout(2);
}

uint8_t EscApdF::update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    return (crc_u);
}

uint8_t EscApdF::get_crc8(uint8_t *buffer, uint8_t lenght)
{
    uint8_t crc = 0, i;
    for (i = 0; i < lenght; i++)
        crc = update_crc8(buffer[i], crc);
    return crc;
}

void EscApdF::update()
{
    uint8_t lenght = serial_.availableTimeout();
    if (lenght == APDF_PACKET_LENGHT || lenght == KISS_PACKET_LENGHT)
    {
        uint8_t data[lenght];
        serial_.readBytes(data, lenght);
        //if (data[10] == 0 && data[11] == 0)
        if (get_crc8(data, KISS_PACKET_LENGHT) == data[9])
        {
            float temp = data[0];
            float voltage = ((uint16_t)data[1] << 8 | data[2]) / 100.0;
            float current = ((uint16_t)data[3] << 8 | data[4]) / 100.0;
            float consumption = ((uint16_t)data[5] << 8 | data[6]);
            float rpm = ((uint16_t)data[7] << 8 | data[8]) * 100.0;
            temp_ = calcAverage(alphaTemp_ / 100.0F, temp_, temp);
            voltage_ = calcAverage(alphaVolt_ / 100.0F, voltage_, voltage);
            current_ = calcAverage(alphaCurr_ / 100.0F, current_, current);
            consumption_ = calcAverage(alphaVolt_ / 100.0F, consumption_, consumption);
            rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
            if (cellCount_ == 255)
                if (millis() > 10000 && voltage_ > 1)
                    cellCount_ = setCellCount(voltage_);
            cellVoltage_ = voltage_ / cellCount_;
#ifdef DEBUG_APDF
            DEBUG_PRINT(millis());
            DEBUG_PRINT(" T:");
            DEBUG_PRINT(temp);
            DEBUG_PRINT(" V:");
            DEBUG_PRINT(voltage);
            DEBUG_PRINT(" C:");
            DEBUG_PRINT(current);
            DEBUG_PRINT(" Cs:");
            DEBUG_PRINT(consumption);
            DEBUG_PRINT(" R:");
            DEBUG_PRINT(rpm);
            DEBUG_PRINTLN();
#endif
        }
    }
#ifdef SIM_SENSORS
    temp_ = 12.34;
    voltage_ = 12.34;
    current_ = 12.34;
    consumption_ = 12.34;
    rpm_ = 12345.67;
    cellVoltage_ = voltage_ / cellCount_;
#endif
}

float *EscApdF::rpmP()
{
    return &rpm_;
}

float *EscApdF::voltageP()
{
    return &voltage_;
}

float *EscApdF::currentP()
{
    return &current_;
}

float *EscApdF::consumptionP()
{
    return &consumption_;
}

float *EscApdF::tempP()
{
    return &temp_;
}

float *EscApdF::cellVoltageP()
{
    return &cellVoltage_;
}
