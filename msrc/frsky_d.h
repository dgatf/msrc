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

class Frsky
{
private:
    AbstractSerial &serial_;
    Sensord *sensorP = NULL;
    void sendByte(uint8_t c, bool header);

public:
    Frsky(AbstractSerial &serial);
    ~Frsky();
    void begin();
    void sendData(uint8_t dataId, uint16_t value);
    void addSensor(Sensord *newSensorP);
    void deleteSensors();
    void update();
    void setConfig(Config &config);
};

#endif
