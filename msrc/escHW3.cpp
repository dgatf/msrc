#include "escHW3.h"

EscHW3::EscHW3(AbstractSerial &serial, uint8_t alphaRpm) : alphaRpm_(alphaRpm), serial_(serial)
{
}

void EscHW3::begin()
{
    serial_.begin(19200, SERIAL_8N1);
    serial_.setTimeout(2);
}

void EscHW3::update()
{
    static uint16_t serialTs = 0;
    if (serial_.availableTimeout() == ESCHWV3_PACKET_LENGHT)
    {
        uint8_t data[ESCHWV3_PACKET_LENGHT];
        serial_.readBytes(data, ESCHWV3_PACKET_LENGHT);
        if (data[0] == 0x9B && data[4] == 0 && data[6] == 0)
        {
            uint16_t rpmCycle = (uint16_t)data[8] << 8 | data[9];
            if (rpmCycle <= 0)
                rpmCycle = 1;
            float rpm = 60000000.0 / rpmCycle;
            rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
#ifdef DEBUG_HW3
            uint32_t pn = (uint32_t)data[1] << 16 | (uint16_t)data[2] << 8 | data[3];
            DEBUG_PRINT(pn);
            DEBUG_PRINT(" R:");
            DEBUG_PRINT(rpm);
            DEBUG_PRINTLN();
#endif
        }
    }
    if ((uint16_t)micros() - serialTs > 50000)
        rpm_ = 0;
#ifdef SIM_SENSORS
    rpm_ = 12345.67;
#endif
}

float *EscHW3::rpmP()
{
    return &rpm_;
}