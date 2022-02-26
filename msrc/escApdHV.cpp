#include "escApdHV.h"

EscApdHV::EscApdHV(AbstractSerial &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp)
{
}

void EscApdHV::begin()
{
    serial_.begin(115200, SERIAL__8N1);
    serial_.setTimeout(2);
}

uint16_t EscApdHV::get_crc16(uint8_t *buffer)
{
    uint16_t fCCRC16;
    uint16_t c0 = 0;
    uint16_t c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (uint8_t i = 0; i < 18; i++) //Check only first 18 bytes, skip crc bytes
    {
        c0 = (uint16_t)(c0 + (buffer[i])) % 255;
        c1 = (uint16_t)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = (c1 << 8) | c0;
    return fCCRC16;
}

void EscApdHV::update()
{
    if (serial_.availableTimeout() == APDHV_PACKET_LENGHT)
    {
        uint8_t data[APDHV_PACKET_LENGHT];
        serial_.readBytes(data, APDHV_PACKET_LENGHT);
        //if (data[20] == 0 && data[21] == 0)
        if (get_crc16(data) == (((uint16_t)data[19] << 8) | data[18]))
        {
            float voltage = ((uint16_t)data[1] << 8 | data[0]) / 100.0;
            float temp = calcTemp((uint16_t)data[3] << 8 | data[2]);
            float current = ((uint16_t)data[5] << 8 | data[4]) / 12.5;
            float rpm = (uint32_t)data[11] << 24 | (uint32_t)data[10] << 16 | (uint16_t)data[9] << 8 | data[8];

            temp_ = calcAverage(alphaTemp_ / 100.0F, temp_, temp);
            voltage_ = calcAverage(alphaVolt_ / 100.0F, voltage_, voltage);
            current_ = calcAverage(alphaCurr_ / 100.0F, current_, current);
            rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
            consumption_ += calcConsumption(current);
            if (cellCount_ == 255)
                if (millis() > 10000 && voltage_ > 1)
                    cellCount_ = setCellCount(voltage_);
            cellVoltage_ = voltage_ / cellCount_;
#ifdef DEBUG_APDHV
            DEBUG_PRINT(millis());
            DEBUG_PRINT(" V:");
            DEBUG_PRINT(voltage);
            DEBUG_PRINT(" T:");
            DEBUG_PRINT(temp);
            DEBUG_PRINT(" C:");
            DEBUG_PRINT(current);
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

float EscApdHV::calcTemp(uint16_t rawVal)
{
    uint16_t SERIESRESISTOR = 10000;
    uint16_t NOMINAL_RESISTANCE = 10000;
    uint8_t NOMINAL_TEMPERATURE = 25;
    uint16_t BCOEFFICIENT = 3455;

    //convert value to resistance
    float Rntc = (4096 / (float)rawVal) - 1;
    Rntc = SERIESRESISTOR / Rntc;

    // Get the temperature
    float temperature = Rntc / (float)NOMINAL_RESISTANCE; // (R/Ro)
    temperature = (float)log(temperature);                // ln(R/Ro)
    temperature /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)

    temperature += (float)1.0 / ((float)NOMINAL_TEMPERATURE + (float)273.15); // + (1/To)
    temperature = (float)1.0 / temperature;                                   // Invert
    temperature -= (float)273.15;
    return temperature;
}

float *EscApdHV::rpmP()
{
    return &rpm_;
}

float *EscApdHV::voltageP()
{
    return &voltage_;
}

float *EscApdHV::currentP()
{
    return &current_;
}

float *EscApdHV::tempP()
{
    return &temp_;
}

float *EscApdHV::consumptionP()
{
    return &consumption_;
}

float *EscApdHV::cellVoltageP()
{
    return &cellVoltage_;
}
