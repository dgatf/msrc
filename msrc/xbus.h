#ifndef XBUS_H
#define XBUS_H

#define XBUS_GPS 0x16
#define XBUS_ESC 0x20
#define XBUS_AIRSPEED 0x11
#define XBUS_ALTIMETER 0x12
#define XBUS_BATTERY 0x34
#define XBUS_RPM_VOLT_TEMP 0x7E

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

struct Xbus_Airspeed
{
    uint8_t identifier = 0x11;
    uint8_t sID = 0;          // Secondary ID
    uint16_t airspeed = 0;    // 1 km/h increments
    uint16_t maxAirspeed = 0; // 1 km/h increments
};

struct Xbus_Battery
{
    uint8_t id = 0x34;    // Source device = 0x34
    uint8_t sID;          // Secondary ID
    int16_t current_A;    // Instantaneous current, 0.1A (0-3276.8A)
    int16_t chargeUsed_A; // Integrated mAh used, 1mAh (0-32.766Ah)
    uint16_t temp_A;      // Temperature, 0.1C (0-150.0C, // 0x7FFF indicates not populated)
    int16_t current_B;    // Instantaneous current, 0.1A (0-6553.4A)
    int16_t chargeUsed_B; // Integrated mAh used, 1mAh (0-65.534Ah)
    uint16_t temp_B;      // Temperature, 0.1C (0-150.0C,// 0x7FFF indicates not populated)
};

struct Xbus_Gps
{
    uint8_t identifier = 0x16; // Source device = 0x16
    uint8_t sID;               // Secondary ID
    uint16_t altitudeLow;      // BCD, meters, format 3.1 (Low bits of alt)
    uint32_t latitude;         // BCD, format 4.4, // Degrees * 100 + minutes, < 100 degrees
    uint32_t longitude;        // BCD, format 4.4, // Degrees * 100 + minutes, flag --> > 99deg
    uint16_t course;           // BCD, 3.1
    uint8_t HDOP;              // BCD, format 1.1
    uint8_t GPSflags;          // see definitions below
};

class Xbus
{
private:
    uint8_t addressMask = 0;
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    static volatile Xbus_Esc xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
    static volatile Xbus_RpmVoltTemp xbusRpmVoltTemp1;
#endif
#if CONFIG_AIRSPEED
    static volatile Xbus_Airspeed xbusAirspeed;
#endif
#if CONFIG_CURRENT
    static volatile Xbus_Battery xbusBattery;
#endif
#if CONFIG_GPS
    static volatile Xbus_Gps xbusGps;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    EscHW3Interface esc = EscHW3Interface(ESC_SERIAL, CONFIG_ALPHA_RPM);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_HV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_HV
    EscHW4Interface esc = EscHW4Interface(ESC_SERIAL, CONFIG_ALPHA_RPM, CONFIG_ALPHA_VOLT, CONFIG_ALPHA_CURR, CONFIG_ALPHA_TEMP, CONFIG_ESC_PROTOCOL - PROTOCOL_HW_V4_LV);
#endif
#if CONFIG_VOLTAGE1
    VoltageInterface volt1 = VoltageInterface(PIN_VOLTAGE1, CONFIG_ALPHA_VOLT);
#endif
#if CONFIG_TEMP1
    NtcInterface ntc1 = NtcInterface(PIN_NTC1, CONFIG_ALPHA_TEMP);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM
    EscPWMInterface escPwm = EscPWMInterface(CONFIG_ALPHA_RPM);
#endif
#if CONFIG_CURRENT
    VoltageInterface curr = VoltageInterface(PIN_CURRENT, CONFIG_ALPHA_CURR);
#endif
#if CONFIG_AIRSPEED
    PressureInterface airspeed = PressureInterface(PIN_PRESSURE, CONFIG_ALPHA_DEF);
#endif
    static void i2c_request_handler();

public:
    Xbus();
    void begin();
    void update();
};

#endif