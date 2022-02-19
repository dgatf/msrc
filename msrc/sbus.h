#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>
#include "softserial.h"
#include "hardserial.h"
#include "sensor.h"
#include "constants.h"

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
#include "bn220.h"
#include "configeeprom.h"
#include "pwmout.h"

#define SBUS_SERIAL_TIMEOUT 1500
#define SBUS_PACKET_LENGHT 25

#define SBUS_WAIT 0
#define SBUS_SEND 1

#define SBUS_SLOT_0_DELAY 2000
#define SBUS_INTER_SLOT_DELAY 660

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
#define ESC_PROTOCOL CONFIG_ESC_PROTOCOL
#else
#define ESC_PROTOCOL config.protocol
#endif

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

#if defined(__AVR_ATmega328P__) ||  defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)
extern void (*TIMER2_COMPA_handlerP)();
#endif

#if defined(__AVR_ATmega32U4__)
extern void (*TIMER3_COMPB_handlerP)();
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
extern void (*FTM0_IRQ_handlerP)();
#endif

class Sbus
{
private:
    AbstractSerial &serial_;
    static uint8_t telemetryPacket ;
    static SensorSbus *sensorSbusP[32];
    static const uint8_t slotId[32];
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
    static void TIMER_COMP_handler();
#endif
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    static void FTM0_IRQ_handler();
#endif
    static void sendSlot(uint8_t number);
    void sendPacket();
    static uint32_t ts2;

public:
    Sbus(AbstractSerial &serial);
    ~Sbus();
    void begin();
    void addSensor(uint8_t slot, SensorSbus *newSensorSbusP);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
};

#endif