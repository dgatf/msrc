#include "escHW3.h"

EscHW3::EscHW3(Stream &serial, uint8_t alphaRpm) : alphaRpm_(alphaRpm), serial_(serial) {}

void EscHW3::update()
{
    static uint16_t serialTs = 0;
    static uint8_t serialCount = 0;
    if (serial_.available() > serialCount)
    {
        serialCount = serial_.available();
        serialTs = micros();
    }
    if (((uint16_t)micros() - serialTs > ESCHWV3_ESCSERIAL_TIMEOUT) && serialCount > 0)
    {
        if (serialCount == ESCHWV3_PACKET_LENGHT)
        {
            uint8_t data[ESCHWV3_PACKET_LENGHT];
            serial_.readBytes(data, ESCHWV3_PACKET_LENGHT);
            if (data[0] == 0x9B && data[4] == 0 && data[6] == 0)
            {
                thr_ = data[5]; // 0-255
                pwm_ = data[7]; // 0-255
                uint16_t rpmCycle = (uint16_t)data[8] << 8 | data[9];
                if (rpmCycle <= 0)
                    rpmCycle = 1;
                float rpm = 60000000.0 / rpmCycle;
                rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
#if defined(DEBUG_ESC_HW_V3) || defined(DEBUG_ESC)
                uint32_t pn = (uint32_t)data[1] << 16 | (uint16_t)data[2] << 8 | data[3];
                DEBUG_SERIAL.print("N:");
                DEBUG_SERIAL.print(pn);
                DEBUG_SERIAL.print(" R:");
                DEBUG_SERIAL.println(rpm);
#endif
            }
        }
        while (serial_.available())
            serial_.read();
        serialCount = 0;
    }
    if ((uint16_t)micros() - serialTs > 50000)
    {
        pwm_ = 0;
        thr_ = 0;
        rpm_ = 0;
    }
#ifdef SIM_SENSORS
    rpm_ = 12345.67;
#endif
}

uint8_t *EscHW3::thrP()
{
    return &thr_;
}

uint8_t *EscHW3::pwmP()
{
    return &pwm_;
}

float *EscHW3::rpmP()
{
    return &rpm_;
}