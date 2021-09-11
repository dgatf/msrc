#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>
#include "sensor.h"
#include "config.h"

#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "escKontronik.h"
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "bn220.h"
#include "config.h"
#include "configeeprom.h"
#include "pwmout.h"

#define SBUS_SERIAL_TIMEOUT 2

#define SBUS_WAIT 0
#define SBUS_SEND 1

/*

Slot mapping

----
0	    rx volt
1	    rpm
2-3	    volt (v1, v2)
4-5	    vario (speed, alt)
6	    temp1
7	    temp2
----
8-10	curr1 (curr, volt, cons)
11-13	curr2 (curr, volt, cons)
14	
15	
----
16-23	gps (speed, altitude, cog, lat1, lat2, lat3, lon1, lon2)
----
24	
25	
26	
27	
28	
29	
30	
31
----	
*/

class Sbus
{
private:
    Stream &serial_;
    SensorSbus *sensorSbusP[32] = {NULL};
#ifdef SOFTWARE_SERIAL
    SoftwareSerial softSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX);
#endif

public:
    Sbus(Stream &serial);
    ~Sbus();
    void begin();
    void sendPacket(uint8_t telemetryPacket);
    void sendSlot(uint8_t number);
    void addSensor(uint8_t slot, SensorSbus *newSensorSbusP);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
};

#endif