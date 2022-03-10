#ifndef HITEC_H
#define HITEC_H

#define HITEC_I2C_ADDRESS 0x08
#define HITEC_TIMEOUT 1000

#define HITEC_FRAME_0X11 0
#define HITEC_FRAME_0X12 1
#define HITEC_FRAME_0X13 2
#define HITEC_FRAME_0X14 3
#define HITEC_FRAME_0X15 4
#define HITEC_FRAME_0X16 5
#define HITEC_FRAME_0X17 6
#define HITEC_FRAME_0X18 7
#define HITEC_FRAME_0X19 8
#define HITEC_FRAME_0X1A 9
#define HITEC_FRAME_0X1B 10

#define HITEC_FRAME_0X11_RX_BATT 0

#define HITEC_FRAME_0X12_GPS_LAT 0
#define HITEC_FRAME_0X12_TIME 1

#define HITEC_FRAME_0X13_GPS_LON 0
#define HITEC_FRAME_0X13_TEMP2 1

#define HITEC_FRAME_0X14_GPS_SPD 0
#define HITEC_FRAME_0X14_GPS_ALT 1
#define HITEC_FRAME_0X14_TEMP1 2

#define HITEC_FRAME_0X15_FUEL 0
#define HITEC_FRAME_0X15_RPM1 1
#define HITEC_FRAME_0X15_RPM2 2

#define HITEC_FRAME_0X16_DATE 0
#define HITEC_FRAME_0X16_TIME 1

#define HITEC_FRAME_0X17_COG 0
#define HITEC_FRAME_0X17_SATS 1
#define HITEC_FRAME_0X17_TEMP3 2
#define HITEC_FRAME_0X17_TEMP4 3

#define HITEC_FRAME_0X18_VOLT 0
#define HITEC_FRAME_0X18_AMP 1

#define HITEC_FRAME_0X19_AMP1 0
#define HITEC_FRAME_0X19_AMP2 1
#define HITEC_FRAME_0X19_AMP3 2
#define HITEC_FRAME_0X19_AMP4 3

#define HITEC_FRAME_0X1A_ASPD 0

#define HITEC_FRAME_0X1B_ALTU 0
#define HITEC_FRAME_0X1B_ALTF 1


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

class Hitec
{
private:
    struct DeviceElement
    {
        AbstractDevice *deviceP;
        DeviceElement *nextP;
    };
    static volatile bool isEnabledFrame[];
    static bool isEmpty;
    static float *frame_0x11_P[];
    static float *frame_0x12_P[];
    static float *frame_0x13_P[];
    static float *frame_0x14_P[];
    static float *frame_0x15_P[];
    static float *frame_0x16_P[];
    static float *frame_0x17_P[];
    static float *frame_0x18_P[];
    static float *frame_0x19_P[];
    static float *frame_0x1A_P[];
    static float *frame_0x1B_P[];
    DeviceElement *deviceElementP = NULL;
    static void i2c_request_handler();
    void setConfig(Config &config);
public:
    Hitec();
    void begin();
    void addDevice(AbstractDevice *deviceP);
    void update();
};

#endif
