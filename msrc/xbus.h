#ifndef XBUS_H
#define XBUS_H

#define XBUS_AIRSPEED 0x11
#define XBUS_ALTIMETER 0x12
#define XBUS_GPS_LOC 0x16
#define XBUS_GPS_STAT 0x17
#define XBUS_ESC 0x20
#define XBUS_BATTERY 0x34
#define XBUS_RPM_VOLT_TEMP 0x7E

#define GPS_INFO_FLAGS_IS_NORTH_BIT 0
#define GPS_INFO_FLAGS_IS_EAST_BIT 1
#define GPS_INFO_FLAGS_LONG_GREATER_99_BIT 2
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT 7

#include <Arduino.h>
#include <Wire.h>
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "bn220.h"
#include "config.h"

struct Xbus_Esc
{
    uint8_t identifier = 0x20; // Source device = 0x20
    uint8_t sID = 0;           // Secondary ID
    uint16_t RPM = 0;          // RPM, 10RPM (0-655340 RPM).0xFFFF -->
    uint16_t voltsInput = 0;   // Volts, 0.01v (0-655.34V).0xFFFF -->
    uint16_t tempFET = 0;      // Temperature, 0.1C (0-999.8C)0xFFFF -->
    uint16_t currentMotor = 0; // Current, 10mA (0-655.34A).0xFFFF -->
    uint16_t tempBEC = 0;      // Temperature, 0.1C (0-999.8C)0x7FFF -->
    uint8_t currentBEC = 0;    // BEC Current, 100mA (0-25.4A). 0xFF ---->
    uint8_t voltsBEC = 0;      // BEC Volts, 0.05V (0-12.70V). 0xFF ---->
    uint8_t throttle = 0;      // 0.5% (0-127%). 0xFF ---->
    uint8_t powerOut = 0;      // Power Output, 0.5% (0-127%). 0xFF ---->
};

struct Xbus_RpmVoltTemp
{
    uint8_t identifier = 0x7E;
    uint8_t sID = 0;
    uint16_t microseconds = 0; // microseconds between pulse leading edges
    uint16_t volts = 0;        // 0.01V increments
    int16_t temperature = 0;   // degrees F
};

struct Xbus_Battery
{
    uint8_t id = 0x34;        // Source device = 0x34
    uint8_t sID = 0;          // Secondary ID
    int16_t current_A = 0;    // Instantaneous current, 0.1A (0-3276.8A)
    int16_t chargeUsed_A = 0; // Integrated mAh used, 1mAh (0-32.766Ah)
    uint16_t temp_A = 0;      // Temperature, 0.1C (0-150.0C, // 0x7FFF indicates not populated)
    int16_t current_B = 0;    // Instantaneous current, 0.1A (0-6553.4A)
    int16_t chargeUsed_B = 0; // Integrated mAh used, 1mAh (0-65.534Ah)
    uint16_t temp_B = 0;      // Temperature, 0.1C (0-150.0C,// 0x7FFF indicates not populated)
};

struct Xbus_Gps_Loc
{
    uint8_t identifier = 0x16; // Source device = 0x16
    uint8_t sID = 0;           // Secondary ID
    uint16_t altitudeLow = 0;  // BCD, meters, format 3.1 (Low bits of alt)
    uint32_t latitude = 0;     // BCD, format 4.4, // Degrees * 100 + minutes, < 100 degrees 1234.1234
    uint32_t longitude = 0;    // BCD, format 4.4, // Degrees * 100 + minutes, flag --> > 99deg
    uint16_t course = 0;       // BCD, 3.1
    uint8_t HDOP = 0;          // BCD, format 1.1
    uint8_t GPSflags = 0;      // see definitions below
};

struct Xbus_Gps_Stat
{
    uint8_t identifier = 0x17;
    uint8_t sID = 0;
    uint16_t speed = 0;       // BCD, knots, format 3.1
    uint32_t UTC = 0;         // BCD, format HH:MM:SS.S, format 6.1
    uint8_t numSats = 0;      // BCD, 0-99
    uint8_t altitudeHigh = 0; // BCD, meters, format 2.0 (High bits alt)
};

class Xbus
{
private:
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
    uint8_t addressMask = 0;
#endif
    static void i2c_request_handler();

protected:
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL ==  PROTOCOL_HW_V4_LV || CONFIG_ESC_PROTOCOL ==  PROTOCOL_HW_V4_HV || CONFIG_ESC_PROTOCOL ==  PROTOCOL_HW_V5_LV || CONFIG_ESC_PROTOCOL ==  PROTOCOL_HW_V5_HV
    static Xbus_Esc xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
    static Xbus_RpmVoltTemp xbusRpmVoltTemp1;
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
    static Xbus_RpmVoltTemp xbusRpmVoltTemp2;
#endif
#if CONFIG_AIRSPEED
    static Xbus_Airspeed xbusAirspeed;
    Pressure airspeed = Pressure(PIN_PRESSURE, CONFIG_ALPHA_DEF);
#endif
#if CONFIG_CURRENT
    static Xbus_Battery xbusBattery;
    Voltage curr = Voltage(PIN_CURRENT, CONFIG_ALPHA_CURR);
#endif
#if CONFIG_GPS
    static Xbus_Gps_Loc xbusGpsLoc;
    static Xbus_Gps_Stat xbusGpsStat;
    Bn220 gps = Bn220(GPS_SERIAL);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM
    EscPWM escPwm = EscPWM(CONFIG_ALPHA_RPM);
#endif
#if CONFIG_VOLTAGE1
    Voltage volt1 = Voltage(PIN_VOLTAGE1, CONFIG_ALPHA_VOLT);
#endif
#if CONFIG_NTC1
    Ntc ntc1 = Ntc(PIN_NTC1, CONFIG_ALPHA_TEMP);
#endif
#if CONFIG_VOLTAGE2
    Voltage volt2 = Voltage(PIN_VOLTAGE2, CONFIG_ALPHA_VOLT);
#endif
#if CONFIG_NTC2
    Ntc ntc2 = Ntc(PIN_NTC2, CONFIG_ALPHA_TEMP);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    EscHW3 esc = EscHW3(ESC_SERIAL, CONFIG_ALPHA_RPM);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_HV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_HV
    EscHW4 esc = EscHW4(ESC_SERIAL, CONFIG_ALPHA_RPM, CONFIG_ALPHA_VOLT, CONFIG_ALPHA_CURR, CONFIG_ALPHA_TEMP, CONFIG_ESC_PROTOCOL - PROTOCOL_HW_V4_LV);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    EscCastle esc = EscCastle(CONFIG_ALPHA_RPM, CONFIG_ALPHA_VOLT, CONFIG_ALPHA_CURR, CONFIG_ALPHA_TEMP);
#endif

public:
    Xbus();
    void begin();
    void update();
    void bcd(uint8_t *output, float value, uint8_t precision);
    void bcd(uint16_t *output, float value, uint8_t precision);
    void bcd(uint32_t *output, float value, uint8_t precision);
};

#endif