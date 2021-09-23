#include "srxl.h"

Srxl::Srxl(Stream &serial) : serial_(serial) {}

void Srxl::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    uint8_t cont = 0;
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 && \
    CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_LV && \
    CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_HV && \
    CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_LV  && \
    CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_HV
    ESC_SERIAL.begin(19200);
    cont++;
    list[cont] = XBUS_ESC;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK
    ESC_SERIAL.begin(115200, SERIAL_8E1);
    cont++;
    list[cont] = XBUS_ESC;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
    cont++;
    list[cont] = XBUS_RPM_VOLT_TEMP;
#endif
#if CONFIG_AIRSPEED
    cont++;
    list[cont] = XBUS_AIRSPEED;
#endif
#if CONFIG_GPS
    GPS_SERIAL.begin(GPS_BAUD_RATE);
    GPS_SERIAL.setTimeout(BN220_TIMEOUT);
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
    buffer[0] = SRXL_HEADER;
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
    crc = __builtin_bswap16(getCrc(buffer, 19)); // all bytes including header
    //crc = __builtin_bswap16(getCrc(buffer + 3, 16));  // only 16bytes (xbus)
    memcpy(buffer + 19, &crc, 2);
    SRXL_IBUS_SBUS_SERIAL.write(buffer, 21);
#ifdef DEBUG
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

void Srxl::updateSrxl()
{
    static uint16_t serialTs = 0;
    static uint8_t serialCount = 0;
    uint8_t status = SRXL_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)millis() - ts > 100)
    {
        if (!mute)
        {
            status = SRXL_SEND;
        }
        mute = !mute;
        ts = millis();
    }
#else
    if (serial_.available() > serialCount)
    {
        serialCount = serial_.available();
        serialTs = micros();
    }
    if ((uint16_t)micros() - serialTs > SRXL_SERIAL_TIMEOUT && serialCount > 0)
    {
        if (serialCount == SRXL_FRAMELEN)
        {
            uint8_t buff[SRXL_FRAMELEN];
            serial_.readBytes(buff, SRXL_FRAMELEN);
            if (buff[0] == SRXL_HEADER)
            {
                if (!mute)
                {
                    status = SRXL_SEND;
                }
                mute = !mute;
            }
        }
        while (serial_.available())
            serial_.read();
        serialCount = 0;
    }
#endif
    if (status == SRXL_SEND)
    {
        send();
    }
    update();
}