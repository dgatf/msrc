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

#define SBUS_SLOT_TEMP1 1
#define SBUS_SLOT_RPM 2
#define SBUS_SLOT_VARIO_SPEED 3
#define SBUS_SLOT_VARIO_ALT 4
#define SBUS_SLOT_VARIO_RESSURE 5
#define SBUS_SLOT_VOLT_V1 6
#define SBUS_SLOT_VOLT_V2 7

#define SBUS_SLOT_GPS_SPD 8
#define SBUS_SLOT_GPS_ALT 9
#define SBUS_SLOT_GPS_TIME 10
#define SBUS_SLOT_GPS_VARIO 11
#define SBUS_SLOT_GPS_LAT1 12
#define SBUS_SLOT_GPS_LAT2 13
#define SBUS_SLOT_GPS_LON1 14
#define SBUS_SLOT_GPS_LON2 15

#define SBUS_SLOT_AIR_SPEED 16

#define SBUS_SLOT_POWER_CURR1 24
#define SBUS_SLOT_POWER_VOLT1 25
#define SBUS_SLOT_POWER_CONS1 26
#define SBUS_SLOT_POWER_CURR2 27
#define SBUS_SLOT_POWER_VOLT2 28
#define SBUS_SLOT_POWER_CONS2 29
#define SBUS_SLOT_TEMP2 30

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
#define ESC_PROTOCOL CONFIG_ESC_PROTOCOL
#else
#define ESC_PROTOCOL config.protocol
#endif

/*          
        Slots mapping

---------------------------------------
0	    RX voltage (reserved)
1       Temp1 (SBS-01T/TE)
2	    RPM (SBS-01RB/RM+/RO)
3-5	    Vario/Altitude (SBS-01A+/02A): 3-vario, 4-altitude, 5-pressure(unused) 
6-7	    Voltage (SBS-01V+): 6-volt1, 7-volt2
---------------------------------------
8-15    GPS (SBS-01G/02G): 8-speed, 9-altitude, 10-time, 11-vario, 12-lat1, 13-lat2, 14-lon1, 15-lon2
---------------------------------------
16      Air speed (SBS-01TAS)
17-23   Unused
---------------------------------------
24-26	Current1 (SBS-01C): 24-current, 25-voltage, 26.consumption 
27-29(+)Current2 (SBS-01C): 24-current, 25-voltage, 26.consumption
30(+)   Temp2 (SBS-01T/TE)
31      Unused
---------------------------------------

(+) Non default slots

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