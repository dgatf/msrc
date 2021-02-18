#include "xbus.h"

#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
Xbus_Esc Xbus::xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
Xbus_RpmVoltTemp Xbus::xbusRpmVoltTemp1;
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
Xbus_RpmVoltTemp Xbus::xbusRpmVoltTemp2;
#endif
#if CONFIG_AIRSPEED
Xbus_Airspeed Xbus::xbusAirspeed;
#endif
#if CONFIG_CURRENT
Xbus_Battery Xbus::xbusBattery;
#endif
#if CONFIG_GPS
Xbus_Gps_Loc Xbus::xbusGpsLoc;
Xbus_Gps_Stat Xbus::xbusGpsStat;
#endif

Xbus::Xbus()
{
}

void Xbus::i2c_request_handler()
{
#ifdef SIM_RX
    uint8_t list[] = {XBUS_AIRSPEED, XBUS_BATTERY, XBUS_ESC, XBUS_GPS_LOC, XBUS_GPS_STAT, XBUS_RPM_VOLT_TEMP};
    static uint8_t cont = 0;
    uint8_t address = list[cont];
    cont++;
    if (cont > 5)
        cont = 0;
#else
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
    uint8_t address = TWDR >> 1;
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    uint8_t address = I2C0_D >> 1;
#endif
#endif
    uint8_t buffer[16] = {0};
    switch (address)
    {
#if CONFIG_AIRSPEED
    case XBUS_AIRSPEED:
        memcpy(buffer, (uint8_t *)&xbusAirspeed, sizeof(xbusAirspeed));
        break;
#endif
#if CONFIG_CURRENT
    case XBUS_BATTERY:
        memcpy(buffer, (uint8_t *)&xbusBattery, sizeof(xbusBattery));
        break;
#endif
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    case XBUS_ESC:
        memcpy(buffer, (uint8_t *)&xbusEsc, sizeof(xbusEsc));
        break;
#endif
#if CONFIG_GPS
    case XBUS_GPS_LOC:
        memcpy(buffer, (uint8_t *)&xbusGpsLoc, sizeof(xbusGpsLoc));
        break;
    case XBUS_GPS_STAT:
        memcpy(buffer, (uint8_t *)&xbusGpsStat, sizeof(xbusGpsStat));
        break;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
    case XBUS_RPM_VOLT_TEMP:
        memcpy(buffer, (uint8_t *)&xbusRpmVoltTemp1, sizeof(xbusRpmVoltTemp1));
        break;
#endif
    default:
        return;
    }
    Wire.write(buffer, 16);
#ifdef DEBUG
    for (int i = 0; i < 16; i++)
    {
        DEBUG_SERIAL.print(buffer[i], HEX);
        DEBUG_SERIAL.print(" ");
    }
    DEBUG_SERIAL.println();
#endif
}

void Xbus::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
    addressMask |= XBUS_RPM_VOLT_TEMP;
#endif
#if CONFIG_AIRSPEED
    addressMask |= XBUS_AIRSPEED;
#endif
#if CONFIG_GPS
    addressMask |= XBUS_GPS_LOC;
    addressMask |= XBUS_GPS_STAT;
    GPS_SERIAL.begin(GPS_BAUD_RATE);
    GPS_SERIAL.setTimeout(BN220_TIMEOUT);
#endif
#if CONFIG_CURRENT
    addressMask |= XBUS_BATTERY;
#endif
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM && CONFIG_ESC_PROTOCOL != PROTOCOL_CASTLE
    addressMask |= XBUS_ESC;
    ESC_SERIAL.begin(19200);
    ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
#endif
    Wire.begin(addressMask);
    Wire.onRequest(i2c_request_handler);
    TWAMR = addressMask << 1;
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    Wire.begin(XBUS_AIRSPEED);
    Wire.onRequest(i2c_request_handler);
    I2C0_RA = XBUS_RPM_VOLT_TEMP << 1;
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
    xbusRpmVoltTemp2.sID = 1;
#endif
}

void Xbus::update()
{
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_HV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_HV
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
    xbusEsc.voltsInput = __builtin_bswap16(*esc.voltageP() * 100);
    xbusEsc.tempFET = __builtin_bswap16(*esc.tempFetP() * 10);
    xbusEsc.currentMotor = __builtin_bswap16(*esc.currentP() * 100);
    xbusEsc.tempBEC = __builtin_bswap16(*esc.tempBecP() * 10);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
    xbusEsc.voltsInput = __builtin_bswap16(*esc.voltageP() * 100);
    xbusEsc.tempFET = __builtin_bswap16(*esc.tempFetP() * 10);
    xbusEsc.currentMotor = __builtin_bswap16(*esc.currentP() * 100);
    xbusEsc.tempBEC = __builtin_bswap16(*esc.tempBecP() * 10);
    xbusEsc.currentBEC = __builtin_bswap16(*esc.becCurrentP() * 10);
    xbusEsc.voltsBEC = __builtin_bswap16(*esc.becVoltageP() * 20);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
    xbusEsc.voltsInput = __builtin_bswap16(*esc.voltageP() * 100);
    xbusEsc.currentMotor = __builtin_bswap16(*esc.currentP() * 100);
    xbusEsc.voltsBEC = *esc.becVoltageP() * 20;
    xbusEsc.currentBEC = *esc.becCurrentP() * 10;
    xbusEsc.tempFET = __builtin_bswap16(*esc.temperatureP() * 10);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM
    escPWM.update();
    xbusRpmVoltTemp1.microseconds = __builtin_bswap16(60000 / *escPWM.rpmP());
#endif
#if CONFIG_VOLTAGE1
    volt1.update();
    xbusRpmVoltTemp1.volts = __builtin_bswap16(*volt1.valueP() * 100);
#endif
#if CONFIG_NTC1
    ntc1.update();
    xbusRpmVoltTemp1.temperature = __builtin_bswap16(*ntc1.valueP());
#endif
#if CONFIG_VOLTAGE2
    volt2.update();
    xbusRpmVoltTemp2.volts = __builtin_bswap16(*volt2.valueP() * 100);
#endif
#if CONFIG_NTC2
    ntc2.update();
    xbusRpmVoltTemp2.temperature = __builtin_bswap16(*ntc2.valueP());
#endif
#if CONFIG_AIRSPEED
    airspeed.update();
    xbusAirspeed.airspeed = __builtin_bswap16(*airspeed.valueP());
#endif
#if CONFIG_CURRENT
    curr.update();
    xbusBattery.current_A = __builtin_bswap16(*curr.valueP() * 100);
#endif
#if CONFIG_GPS
    gps.update();
    float lat = *gps.latP();
    if (lat < 0) // N=1,>0, S=0,<0
        lat *= -1;
    else
        xbusGpsLoc.GPSflags = 1 << GPS_INFO_FLAGS_IS_NORTH_BIT;
    bcd(&xbusGpsLoc.latitude, (uint16_t)(lat / 60) * 100 + fmod(lat, 60), 4);
    float lon = *gps.lonP();
    if (lon < 0) // E=1,>0, W=0,<0
        lon *= -1;
    else
        xbusGpsLoc.GPSflags = 1 << GPS_INFO_FLAGS_IS_EAST_BIT;
    if (lon >= 6000)
    {
        xbusGpsLoc.GPSflags = 1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
        lon -= 6000;
    }
    bcd(&xbusGpsLoc.longitude, (uint16_t)(lon / 60) * 100 + fmod(lon, 60), 4);
    bcd(&xbusGpsLoc.course, *gps.cogP(), 1);
    bcd(&xbusGpsStat.speed, *gps.spdP(), 1);
    bcd(&xbusGpsStat.UTC, *gps.timeP(), 1);
    xbusGpsStat.numSats = *gps.satP();

    float alt = *gps.altP();
    if (alt < 0)
    {
        xbusGpsLoc.GPSflags = 1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT;
        alt *= -1;
    }
    bcd(&xbusGpsLoc.altitudeLow, fmod(alt, 1000), 3);
    bcd(&xbusGpsStat.altitudeHigh, (uint8_t)(alt / 1000), 0);
#endif
#if defined(SIM_RX) && RX_PROTOCOL == RX_XBUS
    static uint32_t timestamp = 0;
    if (millis() - timestamp > 94)
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
    sprintf(buf, "%02i", (uint8_t)value);
    for (int i = 0; i < 2; i++)
        *output |= (buf[i] - 48) << ((1 - i) * 4);
}

void Xbus::bcd(uint16_t *output, float value, uint8_t precision)
{
    char buf[10] = {0};
    *output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%04i", (uint16_t)value);
    for (int i = 0; i < 4; i++)
        *output |= (uint16_t)(buf[i] - 48) << ((3 - i) * 4);
}

void Xbus::bcd(uint32_t *output, float value, uint8_t precision)
{
    char buf[10] = {0};
    *output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%08li", (uint32_t)value);
    for (int i = 0; i < 8; i++)
    {
        *output |= (uint32_t)(buf[i] - 48) << ((7 - i) * 4);
    }
}