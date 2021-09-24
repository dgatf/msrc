#include "escKontronik.h"

EscKontronik::EscKontronik(Stream &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp) {}

void EscKontronik::update()
{
    static uint16_t serialTs = 0;
    static uint8_t serialCount = 0;
    if (serial_.available() > serialCount)
    {
        serialCount = serial_.available();
        serialTs = micros();
    }
    if (((uint16_t)micros() - serialTs > KONTRONIK_ESCSERIAL_TIMEOUT) && serialCount > 0)
    {
        if (serialCount == KONTRONIK_PACKET_LENGHT)
        {
            uint8_t data[KONTRONIK_PACKET_LENGHT];
            serial_.readBytes(data, KONTRONIK_PACKET_LENGHT);
            if (data[0] == 0x4B && data[1] == 0x4F &&data[2] == 0x44 && data[3] == 0x4C)
            {
                float rpm = (uint32_t)data[7] << 24 | (uint32_t)data[6] << 16 | (uint16_t)data[5] << 8 | data[4];
                float voltage = ((uint16_t)data[9] << 8 | data[8]) / 100.0;
                float current = ((uint16_t)data[11] << 8 | data[10]) / 10.0;
                float becCurrent = ((uint16_t)data[19] << 8 | data[18]) / 1000.0;
                float becVoltage = ((uint16_t)data[21] << 8 | data[20]) / 1000.0;
                float tempFet = data[26];
                float tempBec = data[27];
                rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
                voltage_ = calcAverage(alphaVolt_ / 100.0F, voltage_, voltage);
                current_ = calcAverage(alphaCurr_ / 100.0F, current_, current);
                becVoltage_ = calcAverage(alphaVolt_ / 100.0F, becVoltage_, becVoltage);
                becCurrent_ = calcAverage(alphaCurr_ / 100.0F, becCurrent_, becCurrent);
                tempFet_ = calcAverage(alphaTemp_ / 100.0F, tempFet_, tempFet);
                tempBec_ = calcAverage(alphaTemp_ / 100.0F, tempBec_, tempBec);
                if (cellCount_ == 255)
                    if (millis() > 10000 && voltage_ > 1)
                        cellCount_ = setCellCount(voltage_);
                cellVoltage_ = voltage_ / cellCount_;
#if defined(DEBUG_ESC_KONTRONIK) || defined(DEBUG_ESC)
                DEBUG_SERIAL.print("R:");
                DEBUG_SERIAL.print(rpm);
                DEBUG_SERIAL.print(" V:");
                DEBUG_SERIAL.print(voltage);
                DEBUG_SERIAL.print(" C:");
                DEBUG_SERIAL.print(current);
                DEBUG_SERIAL.print(" VB:");
                DEBUG_SERIAL.print(becVoltage);
                DEBUG_SERIAL.print(" CB:");
                DEBUG_SERIAL.print(becCurrent);
                DEBUG_SERIAL.print(" TF:");
                DEBUG_SERIAL.print(tempFet);
                DEBUG_SERIAL.print(" TB:");
                DEBUG_SERIAL.print(tempBec);
                DEBUG_SERIAL.print(" VC:");
                DEBUG_SERIAL.println(cellVoltage_);
#endif
            }
        }
        while (serial_.available())
            serial_.read();
        serialCount = 0;
    }
#ifdef SIM_SENSORS
    rpm_ = 12345.67;
    voltage_ = 12.34;
    current_ = 12.34;
    tempFet_ = 12.34;
    tempBec_ = 12.34;
    cellVoltage_ = voltage_ / cellCount_;
#endif
}

float *EscKontronik::rpmP()
{
    return &rpm_;
}

float *EscKontronik::voltageP()
{
    return &voltage_;
}

float *EscKontronik::currentP()
{
    return &current_;
}

float *EscKontronik::becVoltageP()
{
    return &becVoltage_;
}

float *EscKontronik::becCurrentP()
{
    return &becCurrent_;
}

float *EscKontronik::tempFetP()
{
    return &tempFet_;
}

float *EscKontronik::tempBecP()
{
    return &tempBec_;
}
