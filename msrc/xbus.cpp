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
#if (CONFIG_I2C1_TYPE == I2C_BMP280) && (defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) && defined(I2C_T3_TEENSY)
Xbus_Vario Xbus::xbusVario;
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
    if (cont > sizeof(list))
        cont = 0;
#else
    uint8_t address = 0;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
    address = TWDR >> 1;
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    address = I2C0_D >> 1;
#endif
#if defined(__IMXRT1062__)
    address = LPI2C1_SASR;
#endif
#endif
#ifdef DEBUG_PACKET
    DEBUG_PRINT("<");
    DEBUG_PRINT_HEX(address);
    DEBUG_PRINTLN();
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
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1 || CONFIG_VOLTAGE2 || CONFIG_NTC2
    case XBUS_RPM_VOLT_TEMP:
        static bool sId = 0;
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
        if (!sId)
            memcpy(buffer, (uint8_t *)&xbusRpmVoltTemp1, sizeof(xbusRpmVoltTemp1));
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
        if (sId)
            memcpy(buffer, (uint8_t *)&xbusRpmVoltTemp2, sizeof(xbusRpmVoltTemp2));
        sId = !sId;
#endif
        break;
#endif
#if (CONFIG_I2C1_TYPE == I2C_BMP280) && (defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) && defined(I2C_T3_TEENSY)
    case XBUS_VARIO:
        memcpy(buffer, (uint8_t *)&xbusVario, sizeof(xbusVario));
    break;
#endif
    default:
        return;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    Wire.write(buffer, 16);
    digitalWrite(LED_BUILTIN, LOW);
#ifdef DEBUG
    for (int i = 0; i < 16; i++)
    {
        DEBUG_PRINT_HEX(buffer[i]);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();
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
    gps.begin();
#endif
#if CONFIG_CURRENT
    addressMask |= XBUS_BATTERY;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || \
    CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || \
    CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK || \
    CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    addressMask |= XBUS_ESC;
    esc.begin();
#endif
    Wire.begin(addressMask);
    Wire.onRequest(i2c_request_handler);
    TWAMR = addressMask << 1;
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
#if CONFIG_GPS
    gps.begin();
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || \
    CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || \
    CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK || \
    CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    esc.begin();
#endif
#if (CONFIG_I2C1_TYPE == I2C_BMP280) && !defined(__MK20DX128__) && defined(I2C_T3_TEENSY)
    bmp.begin();
#endif
#if defined(I2C_T3_TEENSY) && !defined(__IMXRT1062__)
    Wire.begin(I2C_SLAVE, XBUS_AIRSPEED, XBUS_RPM_VOLT_TEMP, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
#endif
#if !defined(I2C_T3_TEENSY) && !defined(__IMXRT1062__)
    Wire.begin(XBUS_AIRSPEED);
    I2C0_RA = XBUS_RPM_VOLT_TEMP << 1;
    I2C0_C2 = 1 << 3;
#endif
#if defined(__IMXRT1062__)
    Wire.begin(XBUS_AIRSPEED);
    LPI2C1_SCFGR1 |= LPI2C_SCFGR1_ADDRCFG(B110);
    LPI2C1_SASR |= LPI2C_SAMR_ADDR1(XBUS_RPM_VOLT_TEMP);
#endif
    Wire.onRequest(i2c_request_handler);
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
    xbusRpmVoltTemp2.sID = 1;
#endif

#if defined(XBUS_CLOCK_STRECH_SWITCH)
    pinMode(PIN_SWITCH_XBUS, OUTPUT);
    digitalWrite(PIN_SWITCH_XBUS, LOW);
#endif
}

void Xbus::update()
{
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
    xbusEsc.voltsInput = __builtin_bswap16(*esc.voltageP() * 100);
    xbusEsc.tempFET = __builtin_bswap16(*esc.tempFetP() * 10);
    xbusEsc.currentMotor = __builtin_bswap16(*esc.currentP() * 100);
    xbusEsc.tempBEC = __builtin_bswap16(*esc.tempBecP() * 10);
    pwmOut.setRpmP(esc.rpmP());
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK
    esc.update();
    xbusEsc.RPM = __builtin_bswap16(*esc.rpmP() / 10);
    xbusEsc.voltsInput = __builtin_bswap16(*esc.voltageP() * 100);
    xbusEsc.tempFET = __builtin_bswap16(*esc.tempFetP() * 10);
    xbusEsc.currentMotor = __builtin_bswap16(*esc.currentP() * 100);
    xbusEsc.tempBEC = __builtin_bswap16(*esc.tempBecP() * 10);
    xbusEsc.currentBEC = *esc.becCurrentP() * 10;
    xbusEsc.voltsBEC = *esc.becVoltageP() * 20;
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
    static uint16_t maxSpeed = 0;
    airspeed.update();
    uint16_t speed = round(*airspeed.valueP());
    xbusAirspeed.airspeed = __builtin_bswap16(speed);
    if (speed > maxSpeed)
    {
        maxSpeed = speed;
        xbusAirspeed.maxAirspeed = __builtin_bswap16(speed);
    }
#endif
#if CONFIG_CURRENT
    curr.update();
    xbusBattery.current_A = __builtin_bswap16(*curr.valueP() * 10);
    xbusBattery.chargeUsed_A = __builtin_bswap16(*curr.consumptionP());
#endif
#if CONFIG_GPS
    gps.update();
    uint8_t gpsFlags = 0;
    float lat = *gps.latP();
    if (lat < 0) // N=1,+, S=0,-
        lat *= -1;
    else
        gpsFlags |= 1 << GPS_INFO_FLAGS_IS_NORTH_BIT;
    xbusGpsLoc.latitude = bcd32((uint16_t)(lat / 60) * 100 + fmod(lat, 60), 4);
    float lon = *gps.lonP();
    if (lon < 0) // E=1,+, W=0,-
        lon *= -1;
    else
        gpsFlags |= 1 << GPS_INFO_FLAGS_IS_EAST_BIT;
    if (lon >= 6000)
    {
        gpsFlags |= 1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
        lon -= 6000;
    }
    xbusGpsLoc.longitude = bcd32((uint16_t)(lon / 60) * 100 + fmod(lon, 60), 4);
    xbusGpsLoc.course = bcd16(*gps.cogP(), 1);
    xbusGpsLoc.HDOP = bcd8(*gps.hdopP(), 1);
    xbusGpsStat.speed = bcd16(*gps.spdP(), 1);
    xbusGpsStat.UTC = bcd32(*gps.timeP(), 2);
    xbusGpsStat.numSats = bcd8(*gps.satP(), 0);
    float alt = *gps.altP();
    if (alt < 0)
    {
        gpsFlags |= 1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT;
        alt *= -1;
    }
    xbusGpsLoc.GPSflags = gpsFlags;
    xbusGpsLoc.altitudeLow = bcd16(fmod(alt, 1000), 1);
    xbusGpsStat.altitudeHigh = bcd8((uint8_t)(alt / 1000), 0);
#endif
#if (CONFIG_I2C1_TYPE == I2C_BMP280) && (defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) && defined(I2C_T3_TEENSY)
    bmp.update();
    uint16_t altitude = round(*bmp.altitudeP() * 10);
    uint16_t vario = round(*bmp.varioP() * 10);
    xbusVario.altitude = __builtin_bswap16(altitude);
    xbusVario.delta_0500ms = __builtin_bswap16(vario);
    /*static uint16_t maxAltitude = 0;
    if (altitude > maxAltitude && millis() > 7000)
    {
        maxAltitude = altitude;
        xbusAltitude.maxAltitude = __builtin_bswap16(altitude);
    }*/
#endif
#if defined(SIM_RX) && RX_PROTOCOL == RX_XBUS
    static uint32_t timestamp = 0;
    if (millis() - timestamp > 44)
    {
        i2c_request_handler();
        timestamp = millis();
    }
#endif
}

uint8_t Xbus::bcd8(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint8_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%02i", (uint8_t)value);
    for (int i = 0; i < 2; i++)
        output |= (buf[i] - 48) << ((1 - i) * 4);
    return output;
}

uint16_t Xbus::bcd16(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint16_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%04i", (uint16_t)value);
    for (int i = 0; i < 4; i++)
        output |= (uint16_t)(buf[i] - 48) << ((3 - i) * 4);
    return output;
}

uint32_t Xbus::bcd32(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint32_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%08li", (uint32_t)value);
    for (int i = 0; i < 8; i++)
        output |= (uint32_t)(buf[i] - 48) << ((7 - i) * 4);
    return output;
}