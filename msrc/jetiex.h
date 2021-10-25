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
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "bn220.h"
#include "configeeprom.h"
#include "pwmout.h"

#define JETIEX_MFG_ID_LOW 0x00
#define JETIEX_MFG_ID_HIGH 0xA4
#define JETIEX_DEV_ID_LOW 0x00
#define JETIEX_DEV_ID_HIGH 0xA4
#define JETIEX_PACKET_LENGHT 8
#define JETIEX_WAIT 0
#define JETIEX_SEND 1

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