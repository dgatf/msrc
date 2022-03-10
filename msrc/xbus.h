#ifndef XBUS_H
#define XBUS_H

#define XBUS_AIRSPEED 0x11
#define XBUS_ALTIMETER 0x12
#define XBUS_GPS_LOC 0x16
#define XBUS_GPS_STAT 0x17
#define XBUS_ESC 0x20
#define XBUS_BATTERY 0x34
#define XBUS_VARIO 0X40
#define XBUS_RPM_VOLT_TEMP 0x7E

#define GPS_INFO_FLAGS_IS_NORTH_BIT 0
#define GPS_INFO_FLAGS_IS_EAST_BIT 1
#define GPS_INFO_FLAGS_LONG_GREATER_99_BIT 2
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT 7

#include "constants.h"
#include <Arduino.h>
#include "softserial.h"
#include "hardserial.h"
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "escKontronik.h"
#include "escApdF.h"
#include "escApdHV.h"
#include "voltage.h"
#include "current.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "ms5611.h"
#include "bn220.h"
#include "pwmout.h"

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
    uint8_t identifier = 0x34; // Source device = 0x34
    uint8_t sID = 0;           // Secondary ID
    int16_t current_A = 0;     // Instantaneous current, 0.1A (0-3276.8A)
    int16_t chargeUsed_A = 0;  // Integrated mAh used, 1mAh (0-32.766Ah)
    uint16_t temp_A = 0;       // Temperature, 0.1C (0-150.0C, // 0x7FFF indicates not populated)
    int16_t current_B = 0;     // Instantaneous current, 0.1A (0-6553.4A)
    int16_t chargeUsed_B = 0;  // Integrated mAh used, 1mAh (0-65.534Ah)
    uint16_t temp_B = 0;       // Temperature, 0.1C (0-150.0C,// 0x7FFF indicates not populated)
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
    uint8_t identifier = 0x17; // Source device = 0x17
    uint8_t sID = 0;           // Secondary ID
    uint16_t speed = 0;        // BCD, knots, format 3.1
    uint32_t UTC = 0;          // BCD, format HH:MM:SS.SS, format 6.2
    uint8_t numSats = 0;       // BCD, 0-99
    uint8_t altitudeHigh = 0;  // BCD, meters, format 2.0 (High bits alt)
};

struct Xbus_Airspeed
{
    uint8_t identifier = 0x11; // Source device = 0x11
    uint8_t sID = 0;           // Secondary ID
    uint16_t airspeed = 0;     // 1 km/h increments
    uint16_t maxAirspeed = 0;  // 1 km/h increments
};

struct Xbus_Altitude
{
    uint8_t identifier = 0x12; // Source device = 0x12
    uint8_t sID = 0;           // Secondary ID
    int16_t altitude = 0;      // .1m increments
    int16_t maxAltitude = 0;   // .1m increments
};

struct Xbus_Vario
{
uint8_t identifier = 0x40; // Source device = 0x40
uint8_t sID = 0; // Secondary ID
int16_t altitude; // .1m increments
int16_t delta_0250ms, // delta last 250ms, 0.1m/s increments
delta_0500ms,// delta last 500ms, 0.1m/s increments
delta_1000ms,// delta last 1.0 seconds
delta_1500ms,// delta last 1.5 seconds
delta_2000ms,// delta last 2.0 seconds
delta_3000ms;// delta last 3.0 seconds
};

class Xbus
{
private:
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
    uint8_t addressMask = 0;
#endif
    static void i2c_request_handler();

protected:
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
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
    Pressure airspeed = Pressure(PIN_PRESSURE, 1);
#endif
#if CONFIG_CURRENT
    static Xbus_Battery xbusBattery;
    Current curr = Current(PIN_CURRENT, ALPHA(CONFIG_AVERAGING_ELEMENTS_CURR), CURRENT_MULTIPLIER);
#endif
#if CONFIG_GPS
    static Xbus_Gps_Loc xbusGpsLoc;
    static Xbus_Gps_Stat xbusGpsStat;
    Bn220 gps = Bn220(GPS_SERIAL, GPS_BAUD_RATE);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM
    EscPWM escPWM = EscPWM(ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM));
#endif
#if CONFIG_VOLTAGE1
    Voltage volt1 = Voltage(PIN_VOLTAGE1, ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), VOLTAGE1_MULTIPLIER);
#endif
#if CONFIG_NTC1
    Ntc ntc1 = Ntc(PIN_NTC1, ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP));
#endif
#if CONFIG_VOLTAGE2
    Voltage volt2 = Voltage(PIN_VOLTAGE2, ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), VOLTAGE2_MULTIPLIER);
#endif
#if CONFIG_NTC2
    Ntc ntc2 = Ntc(PIN_NTC2, ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP));
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    EscHW3 esc = EscHW3(ESC_SERIAL, ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM));
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4
    EscHW4 esc = EscHW4(ESC_SERIAL, ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM), ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), ALPHA(CONFIG_AVERAGING_ELEMENTS_CURR), ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP), 0);
    PwmOut pwmOut;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK
    EscKontronik esc = EscKontronik(ESC_SERIAL, ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM), ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), ALPHA(CONFIG_AVERAGING_ELEMENTS_CURR), ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP));
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_APD_F
    EscApdF esc = EscApdF(ESC_SERIAL, ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM), ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), ALPHA(CONFIG_AVERAGING_ELEMENTS_CURR), ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP));
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_APD_HV
    EscApdF esc = EscApdHV(ESC_SERIAL, ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM), ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), ALPHA(CONFIG_AVERAGING_ELEMENTS_CURR), ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP));
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_CASTLE
    EscCastle esc = EscCastle(ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM), ALPHA(CONFIG_AVERAGING_ELEMENTS_VOLT), ALPHA(CONFIG_AVERAGING_ELEMENTS_CURR), ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP));
#endif
#if (CONFIG_I2C1_TYPE == I2C_BMP280) && (defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) && defined(I2C_T3_TEENSY)
    static Xbus_Vario xbusVario;
    Bmp280 bmp = Bmp280(CONFIG_I2C1_ADDRESS, ALPHA(CONFIG_AVERAGING_ELEMENTS_TEMP), ALPHA(1));
#endif

public:
    Xbus();
    void begin();
    void update();
    uint8_t bcd8(float value, uint8_t precision);
    uint16_t bcd16(float value, uint8_t precision);
    uint32_t bcd32(float value, uint8_t precision);
};

#endif