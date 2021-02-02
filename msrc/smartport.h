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

#define LED_SMARTPORT LED_BUILTIN
#define SMARTPORT_TIMEOUT 3

#define SENSOR_ID_1 0x00 // VARIO 0x100 (0 in opentx lua: id - 1)
#define SENSOR_ID_2 0xA1 // FLVSS 0x300
#define SENSOR_ID_3 0x22 // FAS40-S 0x200
#define SENSOR_ID_4 0x83 // GPS 0x800
#define SENSOR_ID_5 0xE4 // RPM 0x500
#define SENSOR_ID_6 0x45 // UART
#define SENSOR_ID_7 0xC6 // UART
#define SENSOR_ID_8 0x67
#define SENSOR_ID_9 0x48
#define SENSOR_ID_10 0xE9
#define SENSOR_ID_11 0x6A
#define SENSOR_ID_12 0xCB
#define SENSOR_ID_13 0xAC
#define SENSOR_ID_14 0xD
#define SENSOR_ID_15 0x8E
#define SENSOR_ID_16 0x2F
#define SENSOR_ID_17 0xD0
#define SENSOR_ID_18 0x71
#define SENSOR_ID_19 0xF2
#define SENSOR_ID_20 0x53
#define SENSOR_ID_21 0x34
#define SENSOR_ID_22 0x95
#define SENSOR_ID_23 0x16
#define SENSOR_ID_24 0xB7 // Accel 0x700
#define SENSOR_ID_25 0x98
#define SENSOR_ID_26 0x39 // Power box
#define SENSOR_ID_27 0xBA // Temp
#define SENSOR_ID_28 0x1B // Fuel 0x600

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


// Config

// byte 1: command

// packet 1: byte 2 version patch, byte 3 version minor, byte 4 version major

// packet 2: 


// i2c
#define I2C_NONE 0
#define I2C_BMP280 1
#define WIRE_TIMEOUT 3

// opentx
#define DATA_ID 0x5000 // DataId (sensor type)

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

class Smartport : public FormatData, public ConfigEeprom
{
private:
    static const uint8_t sensorIdMatrix[29];

    struct Packet
    {
        uint8_t frameId;
        uint16_t dataId;
        uint32_t value;
    };
    Stream &serial_;
    Sensor *sensorP = NULL;
    Packet *packetP = NULL;
    uint8_t sensorId_ = 0;
    uint16_t dataId_ = DATA_ID;
    bool maintenanceMode_ = false;
    void sendByte(uint8_t c, uint16_t *crcp);

public:
    Smartport(Stream &serial);
    ~Smartport();
    void begin();
    uint8_t idToCrc(uint8_t sensorId);
    uint8_t crcToId(uint8_t sensorIdCrc);
    uint8_t read(uint8_t &sensorId, uint8_t &frameId, uint16_t &dataId, uint32_t &value);
    void sendData(uint16_t dataId, uint32_t val);
    void sendData(uint8_t frameId, uint16_t dataId, uint32_t val);
    void sendVoid();
    uint8_t sensorId();
    void setSensorId(uint8_t sensorId);
    void addSensor(Sensor *newSensorP);
    bool addPacket(uint16_t dataId, uint32_t value);
    bool addPacket(uint8_t frameId, uint16_t dataId, uint32_t value);
    void deleteSensors();
    uint8_t update();
    bool isSendPacketReady();
    void setConfig(Config &config);
    void processPacket(uint8_t frameId, uint16_t dataId, uint32_t value);

};

#endif
