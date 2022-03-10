#ifndef JETIEX_H
#define JETIEX_H

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
#include "ms5611.h"
#include "bn220.h"
#include "configeeprom.h"
#include "pwmout.h"

#define JETIEX_TYPE_INT6 0
#define JETIEX_TYPE_INT14 1
#define JETIEX_TYPE_INT22 4
#define JETIEX_TYPE_TIMEDATE 5
#define JETIEX_TYPE_INT30 8
#define JETIEX_TYPE_COORDINATES 9

#define JETIEX_FORMAT_0_DECIMAL 0
#define JETIEX_FORMAT_1_DECIMAL 1
#define JETIEX_FORMAT_2_DECIMAL 2
#define JETIEX_FORMAT_3_DECIMAL 3
#define JETIEX_FORMAT_DATE 1
#define JETIEX_FORMAT_LON 1
#define JETIEX_FORMAT_TIME 0
#define JETIEX_FORMAT_LAT 0

#define JETIEX_MFG_ID_LOW 0x00
#define JETIEX_MFG_ID_HIGH 0xA4
#define JETIEX_DEV_ID_LOW 0x00
#define JETIEX_DEV_ID_HIGH 0xA4
#define JETIEX_PACKET_LENGHT 8
#define JETIEX_WAIT 0
#define JETIEX_SEND 1

#define JETIEX_TIMEOUT 200

class JetiEx
{
private:
    AbstractSerial &serial_;
    SensorJetiEx *sensorJetiExP[16] = {NULL};
    uint32_t baudRate = 125000L;

public:
    JetiEx(AbstractSerial &serial);
    ~JetiEx();
    void begin();
    uint8_t addSensor(SensorJetiEx *sensorJetiExP);
    void update();
    void setConfig(Config &config);
    bool addSensorValueToBuffer(uint8_t *buffer, uint8_t &posBuffer, uint8_t &sensorNumber);
    bool addSensorTextToBuffer(uint8_t *buffer, uint8_t &posBuffer, uint8_t &sensorNumber);
    uint8_t createExBuffer(uint8_t *buffer, bool sendText);
    void sendPacket(uint8_t packetId);
    uint8_t crc8(uint8_t *crc, uint8_t crc_lenght);
    uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
    uint16_t crc16(uint8_t *p, uint16_t len);
    uint16_t update_crc16(uint16_t crc, uint8_t data);

};

#endif