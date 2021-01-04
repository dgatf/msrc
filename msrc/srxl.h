#ifndef SRXL_H
#define SRXL_H

#define escSerial Serial

#define SRXL_GPS 0x16
#define SRXL_ESC 0x20
#define SRXL_AIRSPEED 0x11
#define SRXL_ALTIMETER 0x12
#define SRXL_BATTERY 0x34
#define SRXL_RPM_VOLT_TEMP 0x7E

#include <Arduino.h>
#include <SoftwareSerial.h>
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

struct Srxl_Esc
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

struct Srxl_RpmVoltTemp
{
    uint8_t identifier = 0x7E;
    uint8_t sID = 0;
    uint16_t microseconds = 0; // microseconds between pulse leading edges
    uint16_t volts = 0;        // 0.01V increments
    int16_t temperature = 0;   // degrees F
};

struct Srxl_Airspeed
{
    uint8_t identifier = 0x11;
    uint8_t sID = 0;          // Secondary ID
    uint16_t airspeed = 0;    // 1 km/h increments
    uint16_t maxAirspeed = 0; // 1 km/h increments
};

class Srxl
{
private:
uint8_t list[7] = {0};
SoftwareSerial srxlSerial = SoftwareSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX);
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    Srxl_Esc xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_VOLTAGE2 || CONFIG_NTC1 || CONFIG_NTC2
    Srxl_RpmVoltTemp xbusRpmVoltTemp1;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3
    EscHW3Interface esc = EscHW3Interface(Serial, CONFIG_ALPHA_RPM);
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4_HV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_LV || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V5_HV
    EscHW4Interface esc = EscHW4Interface(Serial, CONFIG_ALPHA_RPM, CONFIG_ALPHA_VOLT, CONFIG_ALPHA_CURR, CONFIG_ALPHA_TEMP, CONFIG_ESC_PROTOCOL - PROTOCOL_HW_V4_LV);
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
public:
    Srxl();
    void begin();
    void update();
    void send();
};

#endif