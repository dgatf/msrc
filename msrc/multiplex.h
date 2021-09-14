#ifndef MULTIPLEX_H
#define MULTIPLEX_H

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

#define MULTIPLEX_SERIAL_TIMEOUT 1000 // us

#define MULTIPLEX_WAIT 0
#define MULTIPLEX_SEND 1


class Multiplex
{
private:
    Stream &serial_;
    SensorMultiplex *sensorMultiplexP[16] = {NULL};
#ifdef SOFTWARE_SERIAL
    SoftwareSerial softSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX);
#endif

public:
    Multiplex(Stream &serial);
    ~Multiplex();
    void begin();
    void sendPacket(uint8_t address);
    void addSensor(SensorMultiplex *newSensorMultiplexP);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
};

#endif