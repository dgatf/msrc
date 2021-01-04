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
    list[cont] = SRXL_RPM_VOLT_TEMP;
#endif
#if CONFIG_AIRSPEED
    cont++;
    list[cont] = SRXL_AIRSPEED;
#endif
#if CONFIG_GPS
    cont++;
    list[cont] = SRXL_GPS;
#endif
#if CONFIG_CURRENT
    cont++;
    list[cont] = SRXL_BATTERY;
#endif
    list[0] = cont;
}

void Srxl::send()
{
    static uint8_t cont = 0;
    uint8_t buffer[16] = {0};
    switch (list[cont + 1]) {
#if CONFIG_AIRSPEED
        case SRXL_AIRSPEED:
             memcpy(buffer, (byte*)&xbusAirspeed, sizeof(xbusAirspeed));
            Wire.write(buffer, 16);
            break;
#endif
#if CONFIG_CURRENT
        case SRXL_BATTERY:
             memcpy(buffer, (byte*)&xbusBattery, sizeof(xbusBattery));
            Wire.write(buffer, 16);
            break;
#endif
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
        case SRXL_ESC:
            memcpy(buffer, (byte*)&xbusEsc, sizeof(xbusEsc));
            Wire.write(buffer, 16);
            break;
#endif
#if CONFIG_GPS
        case SRXL_GPS:
            memcpy(buffer, (byte*)&xbusGps, sizeof(xbusGps));
            Wire.write(buffer, 16);
            break;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
        case SRXL_RPM_VOLT_TEMP:
            memcpy(buffer, (byte*)&xbusRpmVoltTemp1, sizeof(xbusRpmVoltTemp1));
            Wire.write(buffer, 16);
            //Wire.write((byte*)&xbusRpmVoltTemp1, 16);
            break;
#endif
    }
}

void Srxl::update()
{
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    esc.update();
    xbusEsc.RPM = esc.read(0);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_HV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_HV
    esc.update();
    xbusEsc.RPM = esc.read(ESCHW4_RPM);
    xbusEsc.voltsInput = esc.read(ESCHW4_VOLTAGE);
    xbusEsc.currentMotor = esc.read(ESCHW4_CURRENT);
    xbusEsc.tempBEC = esc.read(ESCHW4_TEMPBEC);
    xbusEsc.tempFET = esc.read(ESCHW4_TEMPFET);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    esc.update();
    xbusEsc.RPM = esc.read(CASTLE_RPM);
    xbusEsc.voltsInput = esc.read(CASTLE_VOLTAGE);
    xbusEsc.currentMotor = esc.read(CASTLE_CURRENT);
    xbusEsc.voltsBEC = esc.read(CASTLE_BEC_VOLTAGE);
    xbusEsc.tempBEC = esc.read(CASTLE_TEMP);
    xbusEsc.tempFET = esc.read(CASTLE_TEMP_NTC);
#endif
#if CONFIG_VOLTAGE1
    xbusRpmVoltTemp1.volts = volt1.read(0);
#endif
#if CONFIG_TEMP1
    xbusRpmVoltTemp1.temperature = ntc1.read(0);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM
    xbusRpmVoltTemp1.microseconds = escPwm.read(0);
#endif
#if CONFIG_AIRSPEED
    //xbusRpmVoltTemp1.microseconds = escPwm.read(0);
#endif
    if (srxlSerial.available() >= 18) {
        uint8_t buffer[64];
        static bool rfPacket = false;
        static bool sendPacket = false;
        srxlSerial.readBytesUntil(0xA5, buffer, 64);
        uint8_t result = srxlSerial.readBytes(buffer, 18);
        if (result == 18 && buffer[0] == 0xA5) rfPacket = true;
        if (sendPacket == false) send();
    }
}
