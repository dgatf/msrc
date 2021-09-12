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

#define SBUS_SERIAL_TIMEOUT 2000 // us
#define SBUS_PACKET_LENGHT 25

#define SBUS_WAIT 0
#define SBUS_SEND 1

/*

Slot mapping

----
0	    rx volt (reserved)
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
16-23	gps (speed, altitude, time, vario, lat1, lat2, lon1, lon2)
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
    const uint8_t slotId[32] = {0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                                0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                                0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                                0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB};
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