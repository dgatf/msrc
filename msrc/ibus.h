#ifndef IBUS_H
#define IBUS_H

#define IBUS_TYPE_U16 0
#define IBUS_TYPE_S16 1
#define IBUS_TYPE_U32 2
#define IBUS_TYPE_S32 3

#define IBUS_RECEIVED_NONE 0
#define IBUS_RECEIVED_POLL 1

#define IBUS_COMMAND_DISCOVER 0x8
#define IBUS_COMMAND_TYPE 0x9
#define IBUS_COMMAND_MEASURE 0xA

#define IBUS_TIMEOUT 3

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

class Ibus
{
private:
    Stream &serial_;
    SensorIbus *sensorIbusP[16] = {NULL};
    uint16_t sensorMask = 0B1111111111111110;
    void sendByte(uint8_t c, uint16_t *crcP);
#ifdef SOFTWARE_SERIAL
    SoftwareSerial softSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX);
#endif

public:
    Ibus(Stream &serial);
    ~Ibus();
    void begin();
    void sendData(uint8_t command, uint8_t address);
    void addSensor(SensorIbus *newSensorIbusP);
    void deleteSensors();
    uint8_t read(uint8_t &command, uint8_t &address);
    void update();
    void setConfig(Config &config);
    bool checkCrc(uint8_t *data);
};

#endif
