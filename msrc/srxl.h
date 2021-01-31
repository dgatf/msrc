#ifndef SRXL_H
#define SRXL_H

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
#include "xbus.h"

#define SRXLSERIAL_TIMEOUT 2

#if SRXL_VARIANT == SRXL_V1
#define SRXL_FRAMELEN 27
#endif
#if SRXL_VARIANT == SRXL_V2
#define SRXL_FRAMELEN 35
#endif

// telemetry frame lenght: <0xA5><packet_type=0x80><length=0x15><16-byte telemetry packet><CRC(2bytes)> = 0x15 = 21bytes
// rf packet frame lenght: <0xA5><length=0x12><16-byte telemetry packet><CRC> = 0x12 = 18

#if SRXL_VARIANT == SRXL_V5
#define SRXL_FRAMELEN 18
#endif

class Srxl : public Xbus
{
private:
    Stream &serial_;
    uint8_t list[7] = {0};
#ifdef SOFTWARE_SERIAL
    SoftwareSerial softSerial = SoftwareSerial(PIN_SOFTSERIAL_RX, PIN_SOFTSERIAL_TX);
#endif
    uint16_t getCrc(uint8_t *buffer, uint8_t lenght);
    uint16_t byteCrc(uint16_t crc, uint8_t new_byte);

public:
    Srxl(Stream &serial);
    void begin();
    void checkSerial();
    void send();
};

#endif