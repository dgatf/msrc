#ifndef MULTIPLEX_H
#define MULTIPLEX_H

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

#define MULTIPLEX_SERIAL_TIMEOUT 2000

#define MULTIPLEX_WAIT 0
#define MULTIPLEX_SEND 1

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
#define ESC_PROTOCOL CONFIG_ESC_PROTOCOL
#else
#define ESC_PROTOCOL config.protocol
#endif

class Multiplex
{
private:
    AbstractSerial &serial_;
    SensorMultiplex *sensorMultiplexP[16] = {NULL};

public:
    Multiplex(AbstractSerial &serial);
    ~Multiplex();
    void begin();
    void sendPacket(uint8_t address);
    void addSensor(SensorMultiplex *newSensorMultiplexP);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
};

#endif