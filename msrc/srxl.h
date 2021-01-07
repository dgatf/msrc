#ifndef SRXL_H
#define SRXL_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "bn220.h"
#include "config.h"
#include "xbus.h"

class Srxl : public Xbus
{
private:
    uint8_t list[7] = {0};
    SoftwareSerial srxlSerial = SoftwareSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX);
    uint16_t getCrc(uint8_t *buffer, uint8_t lenght);
    uint16_t byteCrc(uint16_t crc, uint8_t new_byte);

public:
    Srxl();
    void begin();
    void checkSerial();
    void send();
};

#endif