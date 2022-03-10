#ifndef SRXL_H
#define SRXL_H

#include <Arduino.h>
#include "softserial.h"
#include "hardserial.h"
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "escKontronik.h"
#include "voltage.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "ms5611.h"
#include "bn220.h"
#include "constants.h"
#include "xbus.h"

#define SRXL_V1 1
#define SRXL_V2 2
#define SRXL_V5 5

#define SRXL_SERIAL_TIMEOUT 2000
#define SRXL_VARIANT SRXL_V5 // Spektrum

#if SRXL_VARIANT == SRXL_V1
#define SRXL_HEADER 0xA1
#define SRXL_FRAMELEN 27
#endif
#if SRXL_VARIANT == SRXL_V2
#define SRXL_HEADER 0xA2
#define SRXL_FRAMELEN 35
#endif

// srxl v5
// telemetry packet: <0xA5><packet_type=0x80><length=0x15><16-byte telemetry packet><CRC MSB><CRC LSB> = 0x15 = 21
// rf packet:        <0xA5><15-byte payload><CRC MSB><CRC LSB> = 0x12 = 18

#if SRXL_VARIANT == SRXL_V5
#define SRXL_HEADER 0xA5
#define SRXL_FRAMELEN 18
#endif

#define SRXL_WAIT 0
#define SRXL_SEND 1

class Srxl : public Xbus
{
private:
    AbstractSerial &serial_;
    uint8_t list[7] = {0};
    uint16_t getCrc(uint8_t *buffer, uint8_t lenght);
    uint16_t byteCrc(uint16_t crc, uint8_t new_byte);

public:
    Srxl(AbstractSerial &serial);
    void begin();
    void updateSrxl();
    void send();
};

#endif