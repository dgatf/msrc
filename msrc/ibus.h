#ifndef IBUS_H
#define IBUS_H

#define IBUS_TYPE_U16 0
#define IBUS_TYPE_S16 1
#define IBUS_TYPE_U32 2
#define IBUS_TYPE_S32 3
#define IBUS_TYPE_GPS 4

#define IBUS_RECEIVED_NONE 0
#define IBUS_RECEIVED_POLL 1

#define IBUS_COMMAND_DISCOVER 0x8
#define IBUS_COMMAND_TYPE 0x9
#define IBUS_COMMAND_MEASURE 0xA

#define IBUS_TIMEOUT 1000
#define IBUS_PACKET_LENGHT 4

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

class Ibus
{
private:
    AbstractSerial &serial_;
    SensorIbus *sensorIbusP[16] = {NULL};
    /*
     - Sensor at address 0x00 is reserved
     - Sensor at address 0x01 is masked to give enough time to MSRC to init
       Otherwise initial poll may not answered and sensor does not appear
       Poll is every 7ms. If needed, power the receiver after or flash without bootloader
    */
    uint16_t sensorMask = 0B1111111111111100; 
    void sendByte(uint8_t c, uint16_t *crcP);
    void sendData(uint8_t command, uint8_t address);
    uint8_t read(uint8_t &command, uint8_t &address);
    bool checkCrc(uint8_t *data);

public:
    Ibus(AbstractSerial &serial);
    ~Ibus();
    void begin();
    void addSensor(SensorIbus *newSensorIbusP);
    void update();
    void setConfig(Config &config);
};

#endif
