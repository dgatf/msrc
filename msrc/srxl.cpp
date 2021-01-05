#include "srxl.h"

//Srxl::Srxl(Stream &serial) : serial_(serial) {}

Srxl::Srxl() {}

void Srxl::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    srxlSerial.begin(115200);
    srxlSerial.setTimeout(1000);
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE
    esc.begin();
#endif
    uint8_t cont = 0;
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
    cont++;
    list[cont] = XBUS_RPM_VOLT_TEMP;
#endif
#if CONFIG_AIRSPEED
    cont++;
    list[cont] = XBUS_AIRSPEED;
#endif
#if CONFIG_GPS
    cont++;
    list[cont] = XBUS_GPS;
#endif
#if CONFIG_CURRENT
    cont++;
    list[cont] = XBUS_BATTERY;
#endif
    list[0] = cont;
}

uint16_t Srxl::byteCrc (uint16_t crc, uint8_t new_byte)
{
    uint8_t loop;
    crc = crc ^ (uint16_t)new_byte << 8;
    for(loop = 0; loop < 8; loop++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

uint16_t Srxl:: getCrc(uint8_t *buffer, uint8_t lenght) {
    uint16_t crc = 0;
    for (int i = 0; i < lenght; i++ ) crc += byteCrc(crc, buffer[i]);
    return crc;
}

void Srxl::send()
{
    static uint8_t cont = 0;
    uint8_t buffer[20] = {0};
    buffer[0] = 0xA5;
    buffer[1] = 0x80;
    buffer[2] = 0x15;
    switch (list[cont])
    {
#if CONFIG_AIRSPEED
    case XBUS_AIRSPEED:
        memcpy(buffer + 3, (byte *)&Xbus_Airspeed, sizeof(xbusEsc));
        buffer[20] = getCrc(buffer + 3, 16);
        srxlSerial.write(buffer, 20);
        break;
#endif
#if CONFIG_CURRENT
    case XBUS_BATTERY:
        memcpy(buffer + 3, (byte *)&xbusBattery, sizeof(xbusEsc));
        buffer[20] = getCrc(buffer + 3, 16);
        srxlSerial.write(buffer, 20);
        break;
#endif
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    case XBUS_ESC:
        memcpy(buffer + 3, (byte *)&xbusEsc, sizeof(xbusEsc));
        buffer[20] = getCrc(buffer + 3, 16);
        srxlSerial.write(buffer, 20);
        break;
#endif
#if CONFIG_GPS
    case XBUS_GPS_LOC:
        memcpy(buffer + 3, (byte *)&xbusGpsLoc, sizeof(xbusGpsLoc));
        buffer[20] = getCrc(buffer + 3, 16);
        srxlSerial.write(buffer, 20);
        break;
    case XBUS_GPS_STAT:
        memcpy(buffer + 3, (byte *)&xbusGpsStat, sizeof(xbusGpsStat));
        buffer[20] = getCrc(buffer + 3, 16);
        srxlSerial.write(buffer, 20);
        break;
#endif
    case XBUS_RPM_VOLT_TEMP:
        memcpy(buffer + 3, (byte *)&xbusRpmVoltTemp1, sizeof(xbusRpmVoltTemp1));
        buffer[20] = getCrc(buffer + 3, 16);
        srxlSerial.write(buffer, 20);
        break;
    }
    cont++;
    if (cont > list[0]) cont = 0;
}

void Srxl::checkSerial()
{
    if (srxlSerial.available() >= 18)
    {
        uint8_t buffer[64];
        static bool rfPacket = false;
        static bool sendPacket = false;
        srxlSerial.readBytesUntil(0xA5, buffer, 64);
        uint8_t result = srxlSerial.readBytes(buffer, 18);
        if (result == 18 && buffer[0] == 0xA5)
            rfPacket = true;
        if (sendPacket == false)
        {
            sendPacket = true;
            send();
        }
        else
        {
            sendPacket = false;
        }
    }
}