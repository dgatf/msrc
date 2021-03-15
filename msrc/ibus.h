#ifndef IBUS_H
#define IBUS_H

#include <Arduino.h>
#include <SoftwareSerial.h>
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

#define IBUSSERIAL_TIMEOUT 2

struct Request
{
    uint8_t command : 4;
    uint8_t value : 4;
};

class SensorIbus
{
    uint8_t type;
    uint16_t value;

};

class Ibus
{
private:
    Stream &serial_;
#ifdef SOFTWARE_SERIAL
    SoftwareSerial softSerial = SoftwareSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX);
#endif
    uint16_t getCrc(uint8_t *buffer, uint8_t lenght);
    Request read();
    void send();
    void addSensor();

public:
    Ibus(Stream &serial);
    void begin();
    void update();
};

#endif