#include "escHW3.h"

EscHW3Interface::EscHW3Interface(HardwareSerial &serial, uint8_t alphaRpm) : alphaRpm_(alphaRpm), serial_(serial) {}

void EscHW3Interface::begin()
{
    serial_.begin(19200);
    serial_.setTimeout(ESCSERIAL_TIMEOUT);
}

bool EscHW3Interface::update()
{
    static uint16_t tsEsc_ = 0;
    while (serial_.available() >= 10)
    {
        if (serial_.read() == 0x9B)
        {
            uint8_t data[9];
            uint8_t cont = serial_.readBytes(data, 9);
            if (cont == 9 && data[3] == 0 && data[5] == 0)
            {
                thr_ = data[4]; // 0-255
                pwm_ = data[6]; // 0-255
                uint16_t rpmCycle = (uint16_t)data[7] << 8 | data[8];
                if (rpmCycle <= 0)
                    rpmCycle = 1;
                float rpm = (float)60000000UL / rpmCycle;
                rpm_ = calcAverage(alphaRpm_ / 100.0F, rpm_, rpm);
                tsEsc_ = millis();
#ifdef DEBUG_ESC
                uint32_t pn =
                    (uint32_t)data[0] << 16 | (uint16_t)data[1] << 8 | data[2];
                serial_.print("PN: ");
                serial_.print(pn);
                serial_.print(" RPM: ");
                serial_.println(rpm);
#endif
                return true;
            }
        }
    }

    if (rpm_ == 0)
        return false;

    if ((uint16_t)millis() - tsEsc_ > 150)
    {
        pwm_ = 0;
        thr_ = 0;
        rpm_ = 0;
        return true;
    }

    return false;
}

float EscHW3Interface::read(uint8_t index)
{
#ifdef SIM_SENSORS
    if (index == 0)
        return 10000;
    return 0;
#endif
    if (index == 0)
    {
        update();
        return rpm_;
    }
    return 0;
}
