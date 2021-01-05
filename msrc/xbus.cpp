#include "xbus.h"

#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
volatile Xbus_Esc Xbus::xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
volatile Xbus_RpmVoltTemp Xbus::xbusRpmVoltTemp1;
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
volatile Xbus_RpmVoltTemp Xbus::xbusRpmVoltTemp2;
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

Xbus::Xbus()
{
}

void Xbus::i2c_request_handler()
{
#ifdef SIM_RX
    uint8_t list[] = {XBUS_AIRSPEED, XBUS_BATTERY, XBUS_ESC, XBUS_GPS, XBUS_RPM_VOLT_TEMP};
    static uint8_t cont = 0;
    uint8_t address = list[cont];
    cont++;
    if (cont > 4)
        cont = 0;
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
    case XBUS_GPS_LOC:
        memcpy(buffer, (byte *)&xbusGpsLoc, sizeof(xbusGpsLoc));
        Wire.write(buffer, 16);
        break;
    case XBUS_GPS_STAT:
        memcpy(buffer, (byte *)&xbusGpsStat, sizeof(xbusGpsStat));
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
    DEBUG_SERIAL.print(address, HEX);
    DEBUG_SERIAL.print(": ");
    for (int i = 0; i < 16; i++)
    {
        DEBUG_SERIAL.print(buffer[i]);
        DEBUG_SERIAL.print(" ");
    }
    DEBUG_SERIAL.println();

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
    xbusEsc.RPM = esc.read(ESCHW4_RPM) / 10;
    xbusEsc.voltsInput = esc.read(ESCHW4_VOLTAGE) * 100;
    xbusEsc.currentMotor = esc.read(ESCHW4_CURRENT) * 100;
    xbusEsc.tempBEC = esc.read(ESCHW4_TEMPBEC) * 10;
    xbusEsc.tempFET = esc.read(ESCHW4_TEMPFET) * 10;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    esc.update();
    xbusEsc.RPM = esc.read(CASTLE_RPM);
    xbusEsc.voltsInput = esc.read(CASTLE_VOLTAGE) * 100;
    xbusEsc.currentMotor = esc.read(CASTLE_CURRENT) * 100;
    xbusEsc.voltsBEC = esc.read(CASTLE_BEC_VOLTAGE) * 20;
    xbusEsc.tempBEC = esc.read(CASTLE_TEMP) * 10;
    xbusEsc.tempFET = esc.read(CASTLE_TEMP_NTC) * 10;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM
    xbusRpmVoltTemp1.microseconds = 60000 / escPwm.read(0);
#endif
#if CONFIG_VOLTAGE1
    xbusRpmVoltTemp1.volts = volt1.read(0) * 100;
#endif
#if CONFIG_TEMP1
    xbusRpmVoltTemp1.temperature = ntc1.read(0);
#endif
#if CONFIG_VOLTAGE2
    xbusRpmVoltTemp2.volts = volt2.read(0) * 100;
#endif
#if CONFIG_TEMP2
    xbusRpmVoltTemp2.temperature = ntc2.read(0);
#endif
#if CONFIG_AIRSPEED
    xbusRpmVoltTemp1.microseconds = airspeed.read(0);
#endif
#if CONFIG_CURRENT
    xbusBattery.current_A = curr.read(0) * 100;
#endif
#if CONFIG_GPS
    bcd(xbusGpsLoc.latitude, gps.read(BN220_LAT) / 60 * 100 + gps.read(BN220_LAT) % 60, 4);
    xbusGps.GPSflags = (gps.read(BN220_LAT_SIGN) ? 1 : 0) << GPS_INFO_FLAGS_IS_NORTH_BIT; // N=1->1, S=-1->0
    if (gps.read(BN220_LON) >= 100) {
        xbusGps.GPSflags = 1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
        bcd(xbusGpsLoc.longitude, gps.read(BN220_LON) - 100, 4);
    }
    else {
        bcd(xbusGpsLoc.longitude, gps.read(BN220_LON) / 60 * 100 + gps.read(BN220_LON) % 60, 4);
    }
    xbusGps.GPSflags = (gps.read(BN220_LON_SIGN) ? 1 : 0) << GPS_INFO_FLAGS_IS_EAST_BIT;  // E=1->1, W=-1->0
    bcd(xbusGpsLoc.course, gps.read(BN220_COG), 1);
    bcd(xbusGpsStat.speed, gps.read(BN220_SPD), 1);
    bcd(xbusGpsStat.UTC, gps.read(BN220_TIME), 1);
    xbusGpsStat.numSats = gps.read(BN220_SAT);
    xbusGpsLoc = bcd(gps.read(BN220_ALT) % 1000, 3);
    if (gps.read(BN220_ALT) < 0) {
        xbusGps.GPSflags = 1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
        bcd(xbusGpsStat, - gps.read(BN220_ALT) / 1000);
    }
    else
        bcd(xbusGpsStat, gps.read(BN220_ALT) / 1000);
#endif
#ifdef SIM_RX
    static uint32_t timestamp = 0;
    if (millis() - timestamp > 1000)
    {
        i2c_request_handler();
        timestamp = millis();
    }
#endif
}

void Xbus::bcd(uint8_t *output, float value, uint8_t precision)
{
    char buf[10] = {0};
    *output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    itoa((uint8_t)value, buf, 10);
    for (int i = 0; i < 2; i++)
        *output |= (buf[i] - 48) << ((1 - i) * 4);
}

void Xbus::bcd(uint16_t *output, float value, uint8_t precision)
{
    char buf[10] = {0};
    *output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    itoa((uint16_t)value, buf, 10);
    for (int i = 0; i < 4; i++)
        *output |= (buf[i] - 48) << ((3 - i) * 4);
}

void Xbus::bcd(uint32_t *output, float value, uint8_t precision)
{
    char buf[10] = {0};
    *output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    itoa((uint32_t)value, buf, 10);
    for (int i = 0; i < 8; i++)
        *output |= (buf[i] - 48) << ((7 - i) * 4);
}