#include "escHW4.h"

EscHW4::EscHW4(Stream &serial, uint8_t alphaRpm, uint8_t alphaVolt, uint8_t alphaCurr, uint8_t alphaTemp, uint8_t type) : serial_(serial), alphaRpm_(alphaRpm), alphaVolt_(alphaVolt), alphaCurr_(alphaCurr), alphaTemp_(alphaTemp), type_(type) {}

/*
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
#ifdef ESC_SIGNATURE
                memcpy(&signature_[0], &data[1], 10);
#endif
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
                    if (thr_ > 1024 || // try to filter invalid data frames
                        pwm_ > 1024 ||
                        rpm > 200000 ||
                        data[10] & 0xF0 || // for sensors, ADC is 12bits- > higher bits must be 0
                        data[12] & 0xF0 ||
                        data[14] & 0xF0 ||
                        data[16] & 0xF0)
                    {
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
                        DEBUG_SERIAL.println("BF");
#endif
                        return;
                    }
                    uint16_t rawCur = (uint16_t)data[12] << 8 | data[13];
                    if (rawCurrentOffset_ == -1 && rawCur > 0)
                    {
                        rawCurrentOffset_ = rawCur;
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
                        DEBUG_SERIAL.print("CO: ");
                        DEBUG_SERIAL.println(rawCurrentOffset_);
#endif
                    }
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
#ifdef ESC_SIGNATURE
                    signature_[10] = data[13];
                    signature_[11] = data[12];
#endif
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
*/

void EscHW4::update()
{
    static uint16_t serialTs = 0;
    static uint8_t serialCount = 0;
    if (serial_.available() > serialCount)
    {
        serialCount = serial_.available();
        serialTs = micros();
    }
    if (serialCount >= ESCHWV4_PACKET_LENGHT)
    {
        uint8_t data[ESCHWV4_PACKET_LENGHT];
        serial_.readBytes(data, ESCHWV4_PACKET_LENGHT);
        serialCount = serial_.available();
        thr_ = (uint16_t)data[4] << 8 | data[5]; // 0-1024
        pwm_ = (uint16_t)data[6] << 8 | data[7]; // 0-1024
        float rpm = (uint32_t)data[8] << 16 | (uint16_t)data[9] << 8 | data[10];
        if (thr_ < 1024 || // try to filter invalid data frames
            pwm_ < 1024 ||
            rpm < 200000 ||
            data[11] < 0xF || // for sensors, ADC is 12bits- > higher bits must be 0
            data[13] < 0xF ||
            data[15] < 0xF ||
            data[17] < 0xF)
        {
            uint16_t rawCur = (uint16_t)data[13] << 8 | data[14];
            if (rawCurrentOffset_ == -1 && rawCur > 0)
            {
                rawCurrentOffset_ = rawCur;
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
                DEBUG_SERIAL.print("CO:");
                DEBUG_SERIAL.println(rawCurrentOffset_);
#endif
            }
            float voltage = calcVolt((uint16_t)data[11] << 8 | data[12]);
            float current = calcCurr((uint16_t)data[13] << 8 | data[14]);
            float tempFET = calcTemp((uint16_t)data[15] << 8 | data[16]);
            float tempBEC = calcTemp((uint16_t)data[17] << 8 | data[18]);
            rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
            voltage_ = calcAverage(alphaVolt_ / 100.0F, voltage_, voltage);
            current_ = calcAverage(alphaCurr_ / 100.0F, current_, current);
            tempFet_ = calcAverage(alphaTemp_ / 100.0F, tempFet_, tempFET);
            tempBec_ = calcAverage(alphaTemp_ / 100.0F, tempBec_, tempBEC);
            if (cellCount_ == 255)
                if (millis() > 10000 && voltage_ > 1)
                    cellCount_ = setCellCount(voltage_);
            cellVoltage_ = voltage_ / cellCount_;
#ifdef ESC_SIGNATURE
            signature_[10] = data[14];
            signature_[11] = data[13];
#endif
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
            //uint32_t pn =
            //    (uint32_t)data[1] << 16 | (uint16_t)data[2] << 8 | data[3];
            //DEBUG_SERIAL.println(pn);
            DEBUG_SERIAL.print("R:");
            DEBUG_SERIAL.print(rpm);
            DEBUG_SERIAL.print(" V:");
            DEBUG_SERIAL.print(voltage);
            DEBUG_SERIAL.print(" C:");
            DEBUG_SERIAL.print(current);
            DEBUG_SERIAL.print(" T:");
            DEBUG_SERIAL.print(tempFET);
            //DEBUG_SERIAL.print(" ");
            //DEBUG_SERIAL.println(tempBEC);
#endif
        }
    }
#ifdef ESC_SIGNATURE
    if (serialCount == ESCHWV4_SIGNATURE_LENGHT && ((uint16_t)micros() - serialTs > ESCHW4_ESCSERIAL_TIMEOUT))
    {
        uint8_t data[ESCHWV4_SIGNATURE_LENGHT];
        serial_.readBytes(data, ESCHWV4_SIGNATURE_LENGHT);
        serialCount = 0;
        if (data[0] == 0x9B && data[1] == 0x9B)
        {
            memcpy(&signature_[0], &data[2], 10);
#if defined(DEBUG_ESC_HW_V4) || defined(DEBUG_ESC)
            DEBUG_SERIAL.print("S:");
            for (uint8_t i = 0; i < 12; i++)
            {
                DEBUG_SERIAL.print(signature_[i], HEX);
                DEBUG_SERIAL.print(" ");
            }
            DEBUG_SERIAL.println();
#endif
        }
    }
#endif
    if (((uint16_t)micros() - serialTs > ESCHW4_ESCSERIAL_TIMEOUT) && serialCount > 0)
    {
        DEBUG_SERIAL.println(serial_.available());
        while (serial_.available())
            serial_.read();
        serialCount = 0;
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
    if (ntcR_Rref < 0.001)
        return 0;
    float temperature = 1 / (log(ntcR_Rref) / ESCHW4_NTC_BETA + 1 / 298.15) - 273.15;
    if (temperature < 0)
        return 0;
    return temperature;
}

float EscHW4::calcCurr(uint16_t currentRaw)
{
    if (currentRaw - rawCurrentOffset_ < 0)
        return 0;
    return (currentRaw - rawCurrentOffset_) * ESCHW4_V_REF / (ampGain_[type_] * ESCHW4_DIFFAMP_SHUNT * ESCHW4_ADC_RES);
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

float *EscHW4::cellVoltageP()
{
    return &cellVoltage_;
}

#ifdef ESC_SIGNATURE
float *EscHW4::signatureP()
{
    return (float *)&signature_[0];
}
#endif