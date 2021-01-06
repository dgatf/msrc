#include "srxl.h"

//Srxl::Srxl(Stream &serial) : serial_(serial) {}

Srxl::Srxl() {}

void Srxl::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    srxlSerial.begin(115200);
    srxlSerial.setTimeout(2);
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
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
    list[cont] = XBUS_GPS_LOC;
    cont++;
    list[cont] = XBUS_GPS_STAT;
#endif
#if CONFIG_CURRENT
    cont++;
    list[cont] = XBUS_BATTERY;
#endif
    list[0] = cont;
}

uint16_t Srxl::byteCrc(uint16_t crc, uint8_t new_byte)
{
    uint8_t loop;
    crc = crc ^ (uint16_t)new_byte << 8;
    for (loop = 0; loop < 8; loop++)
    {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

uint16_t Srxl::getCrc(uint8_t *buffer, uint8_t lenght)
{
    uint16_t crc = 0;
    for (int i = 0; i < lenght; i++)
        crc += byteCrc(crc, buffer[i]);
    return crc;
}

void Srxl::send()
{
    static uint8_t cont = 1;
    uint8_t buffer[21] = {0};
    buffer[0] = 0xA5;
    buffer[1] = 0x80;
    buffer[2] = 0x15;
    uint16_t crc;
    switch (list[cont])
    {
#if CONFIG_AIRSPEED
    case XBUS_AIRSPEED:
        memcpy(buffer + 3, (uint8_t *)&xbusAirspeed, sizeof(xbusAirspeed));
        break;
#endif
#if CONFIG_CURRENT
    case XBUS_BATTERY:
        memcpy(buffer + 3, (uint8_t *)&xbusBattery, sizeof(xbusBattery));
        break;
#endif
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    case XBUS_ESC:
        memcpy(buffer + 3, (uint8_t *)&xbusEsc, sizeof(xbusEsc));
        break;
#endif
#if CONFIG_GPS
    case XBUS_GPS_LOC:
        memcpy(buffer + 3, (uint8_t *)&xbusGpsLoc, sizeof(xbusGpsLoc));
        break;
    case XBUS_GPS_STAT:
        memcpy(buffer + 3, (uint8_t *)&xbusGpsStat, sizeof(xbusGpsStat));
        break;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
    case XBUS_RPM_VOLT_TEMP:
        memcpy(buffer + 3, (uint8_t *)&xbusRpmVoltTemp1, sizeof(xbusRpmVoltTemp1));
        break;
#endif
    default:
        return;
    }
    crc = __builtin_bswap16(getCrc(buffer + 3, 16));
    memcpy(buffer + 19, &crc, 2);
    srxlSerial.write(buffer, 21);
#ifdef DEBUG
    DEBUG_SERIAL.print("TM send: ");
    DEBUG_SERIAL.println(millis());
    for (int i = 0; i < 21; i++)
    {
        DEBUG_SERIAL.print(buffer[i], HEX);
        DEBUG_SERIAL.print(" ");
    }
    DEBUG_SERIAL.println();
#endif
    cont++;
    if (cont > list[0])
        cont = 1;
}

void Srxl::checkSerial()
{
#ifdef SIM_RX
    static uint32_t timestamp = 0;
    if (millis() - timestamp > 1000)
    {
        send();
        timestamp = millis();
    }
#else
    if (srxlSerial.available())
    {
        static uint32_t timestamp = 0;
        uint8_t buffer[20];
        static bool mute = false;
        uint8_t lenght = srxlSerial.readBytes(buffer, 20);
        if (lenght == 20 && buffer[0] == 0xA5 && buffer[1] == 0x12)
        {
#ifdef DEBUG
            DEBUG_SERIAL.print("RF received: ");
            DEBUG_SERIAL.println(millis());
        for (int i = 0; i < lenght; i++)
        {
            DEBUG_SERIAL.print(buffer[i], HEX);
            DEBUG_SERIAL.print(" ");
        }
        DEBUG_SERIAL.println();
#endif
            if (mute)
                mute = false;
            else
            {
                timestamp = millis();
                delay(1);
                while (srxlSerial.available() && millis() - timestamp < 8000)
                {
                    char buf[21];
                    uint8_t lenght = srxlSerial.readBytes(buf, 21);
                    if (lenght == 21 && !srxlSerial.available())
                        delay(1);
                }
                send();
                mute = true;
            }
        }
    }
#endif
}