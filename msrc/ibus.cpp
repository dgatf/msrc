#include "ibus.h"

Ibus::Ibus(Stream &serial) : serial_(serial) {}

void Ibus::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    ESC_SERIAL.begin(19200);
    ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);

#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
#endif
#if CONFIG_AIRSPEED
#endif
#if CONFIG_GPS
    GPS_SERIAL.begin(GPS_BAUD_RATE);
    GPS_SERIAL.setTimeout(BN220_TIMEOUT);
#endif
#if CONFIG_CURRENT
#endif
}

uint16_t Ibus::getCrc(uint8_t *buffer, uint8_t lenght)
{
    uint16_t crc = 0;
    for (int i = 0; i < lenght; i++)
        crc += buffer[i];
    return 0xFFFF - crc;
}

void Ibus::send()
{

}

Request Ibus::read()
{
#ifdef SIM_RX
    static uint32_t timestamp = 0;
    if (millis() - timestamp > 1000)
    {
        send();
        timestamp = millis();
    }
#else
    Request request;
    request.command = 0;
    request.value = 0;
    if (SMARTPORT_SRXL_FRSKY_SERIAL.available() == 4)
    {
        uint8_t data[4];
        uint8_t lenght = SMARTPORT_SRXL_FRSKY_SERIAL.readBytes(data, 4);
        if (lenght == 4)
        {
#ifdef DEBUG
            DEBUG_SERIAL.print("Packet received: ");
            DEBUG_SERIAL.println(millis());
            for (int i = 0; i < lenght; i++)
            {
                DEBUG_SERIAL.print(data[i], HEX);
                DEBUG_SERIAL.print(" ");
            }
            DEBUG_SERIAL.println();
#endif
            uint16_t crc = (uint16_t)data[3] << 8 | data[2];

            if (crc == getCrc(data, 2))
                memcpy(data + 1, (uint8_t *)&request, 1);
            ;
        }
    }
    return request;
#endif
}

void Ibus::update()
{
}