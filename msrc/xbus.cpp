#include "xbus.h"

#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
volatile Xbus_Esc Xbus::xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
volatile Xbus_RpmVoltTemp Xbus::xbusRpmVoltTemp1;
#endif
#if CONFIG_AIRSPEED
volatile Xbus_Airspeed Xbus::xbusAirspeed;
#endif
#if CONFIG_CURRENT
volatile Xbus_Battery Xbus::xbusBattery;
#endif
#if CONFIG_GPS
volatile Xbus_Gps Xbus::xbusGps;
#endif

Xbus::Xbus() {}

void Xbus::i2c_request_handler()
{
#ifdef SIM_RX
    uint8_t list[] = {XBUS_AIRSPEED, XBUS_BATTERY, XBUS_ESC, XBUS_GPS, XBUS_RPM_VOLT_TEMP};
    static uint8_t cont = 0;
    uint8_t address = list[cont];
    cont++;
    if (cont > 4) cont = 0;
#else
    uint8_t address = TWDR >> 1;
#endif
    uint8_t buffer[16] = {0};
    switch (address)
    {
#if CONFIG_AIRSPEED
    case XBUS_AIRSPEED:
        memcpy(buffer, (byte *)&Xbus_Airspeed, sizeof(xbusEsc));
        Wire.write(buffer, 16);
        break;
#endif
#if CONFIG_CURRENT
    case XBUS_BATTERY:
        memcpy(buffer, (byte *)&xbusBattery, sizeof(xbusEsc));
        Wire.write(buffer, 16);
        break;
#endif
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    case XBUS_ESC:
        memcpy(buffer, (byte *)&xbusEsc, sizeof(xbusEsc));
        Wire.write(buffer, 16);
        break;
#endif
#if CONFIG_GPS
    case XBUS_GPS:
        memcpy(buffer, (byte *)&xbusGps, sizeof(xbusGps));
        Wire.write(buffer, 16);
        break;
#endif
    case XBUS_RPM_VOLT_TEMP:
        memcpy(buffer, (byte *)&xbusRpmVoltTemp1, sizeof(xbusRpmVoltTemp1));
        Wire.write(buffer, 16);
        //Wire.write((byte*)&xbusRpmVoltTemp1, 16);
        break;
    }
#ifdef DEBUG
    for (int i= 0;16;i++) {
        DEBUG_SERIAL.print(buffer[i]);
    }

#endif
}

void Xbus::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Wire.begin(addressMask);
    Wire.onRequest(i2c_request_handler);
    TWAMR = addressMask << 1;
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    esc.begin();
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
    addressMask |= XBUS_RPM_VOLT_TEMP;
#endif
#if CONFIG_AIRSPEED
    addressMask |= XBUS_AIRSPEED;
#endif
#if CONFIG_GPS
    addressMask |= XBUS_GPS;
#endif
#if CONFIG_CURRENT
    addressMask |= XBUS_BATTERY;
#endif
}

void Xbus::update()
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
    xbusRpmVoltTemp1.microseconds = airspeed.read(0);
#endif
}
