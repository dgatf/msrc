/*
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Arduino library to communicate with the Frsky Smartport protocol
 * 
 * 
 */

#ifndef FRSKYD_H
#define FRSKYD_H

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

class Frsky
{
private:
    Stream &serial_;
    Sensord *sensorP = NULL;
    void sendByte(uint8_t c, bool header);
#ifdef SOFTWARE_SERIAL
    SoftwareSerial softSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX);
#endif

public:
    Frsky(Stream &serial);
    ~Frsky();
    void begin();
    void sendData(uint8_t dataId, uint16_t value);
    void addSensor(Sensord *newSensorP);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
};

#endif
