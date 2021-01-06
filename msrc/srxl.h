#ifndef SRXL_H
#define SRXL_H

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
#include "xbus.h"

class Srxl : public Xbus
{
private:
    uint8_t list[7] = {0};
    SoftwareSerial srxlSerial = SoftwareSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX);
#if CONFIG_ESC_PROTOCOL != PROTOCOL_NONE && CONFIG_ESC_PROTOCOL != PROTOCOL_PWM
    Xbus_Esc xbusEsc;
#endif
#if CONFIG_ESC_PROTOCOL == PROTOCOL_PWM || CONFIG_VOLTAGE1 || CONFIG_NTC1
    Xbus_RpmVoltTemp xbusRpmVoltTemp1;
#endif
#if CONFIG_VOLTAGE2 || CONFIG_NTC2
    Xbus_RpmVoltTemp xbusRpmVoltTemp2;
#endif
#if CONFIG_AIRSPEED
    Xbus_Airspeed xbusAirspeed;
#endif
#if CONFIG_CURRENT
    Xbus_Battery xbusBattery;
#endif
#if CONFIG_GPS
    Xbus_Gps_Loc xbusGpsLoc;
    Xbus_Gps_Stat xbusGpsStat;
#endif
    uint16_t getCrc(uint8_t *buffer, uint8_t lenght);
    uint16_t byteCrc (uint16_t crc, uint8_t new_byte);

public:
    Srxl();
    void begin();
    void checkSerial();
    void send();
};

#endif