/*
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Arduino library to communicate with the Frsky Smartport protocol
 * 
 * 
 */

#ifndef SMARTPORT_H
#define SMARTPORT_H

/* Sensor id (lua_id = id - 1)

1 0x00 // VARIO 0x100 
2 0xA1 // FLVSS 0x300
3 0x22 // FAS40-S 0x200
4 0x83 // GPS 0x800
5 0xE4 // RPM 0x500
6 0x45 // UART
7 0xC6 // UART
8 0x67
9 0x48
10 0xE9
11 0x6A
12 0xCB
13 0xAC
14 0xD
15 0x8E
16 0x2F
17 0xD0
18 0x71
19 0xF2
20 0x53
21 0x34
22 0x95
23 0x16
24 0xB7 // Accel 0x700
25 0x98
26 0x39 // Power box
27 0xBA // Temp
28 0x1B // Fuel 0x600
*/

#define SMARTPORT_TIMEOUT 2000
#define RECEIVED_NONE 0
#define RECEIVED_POLL 1
#define RECEIVED_PACKET 2
#define SENT_TELEMETRY 3
#define SENT_VOID 4
#define SENT_PACKET 5
#define SENT_NONE 6
#define MAINTENANCE_ON 7
#define MAINTENANCE_OFF 8
#define SENT_SENSOR_ID 9
#define CHANGED_SENSOR_ID 10

#include <Arduino.h>
#include "softserial.h"
#include "hardserial.h"
#include "sensor.h"
#include "constants.h"
#include "serial.h"

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
#include "configeeprom.h"
#include "pwmout.h"

class Smartport : public FormatData, public ConfigEeprom
{
private:
    static const uint8_t sensorIdMatrix[29];
    Sensor *spSensorP;

    struct Packet
    {
        uint8_t frameId;
        uint16_t dataId;
        uint32_t value;
        Packet *nextP = NULL;
    };
    AbstractSerial &serial_;
    Sensor *sensorP = NULL;
    Packet *packetP = NULL;
    uint8_t sensorId_ = 0;
    uint16_t dataId_ = DATA_ID;
    bool maintenanceMode_ = false;
    void sendByte(uint8_t c, uint16_t *crcp);

public:
    Smartport(AbstractSerial &serial);
    ~Smartport();
    void begin();
    uint8_t idToCrc(uint8_t sensorId);
    uint8_t crcToId(uint8_t sensorIdCrc);
    uint8_t getCrc(uint8_t *data);
    uint8_t read(uint8_t &sensorId, uint8_t &frameId, uint16_t &dataId, uint32_t &value);
    void sendData(uint16_t dataId, uint32_t val);
    void sendData(uint8_t frameId, uint16_t dataId, uint32_t val);
    void sendVoid();
    uint8_t sensorId();
    void setSensorId(uint8_t sensorId);
    void addSensor(Sensor *newSensorP);
    void addPacket(uint16_t dataId, uint32_t value);
    void addPacket(uint8_t frameId, uint16_t dataId, uint32_t value);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
    void processPacket(uint8_t frameId, uint16_t dataId, uint32_t value);

};

#endif
