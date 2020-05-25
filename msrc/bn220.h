#ifndef BN220_H
#define BN220_H

//#define DEBUG_GPS

#define BN220_UNK 0
#define BN220_GGA 1
#define BN220_GLL 2
#define BN220_RMC 3
#define BN220_VTG 4

#define BN220_LAT 0
#define BN220_LON 1
#define BN220_ALT 2
#define BN220_SPD 3
#define BN220_COG 4
#define BN220_KPH 5
#define BN220_SAT 6
#define BN220_DATE 7
#define BN220_TIME 8
#define BN220_LAT_SIGN 9
#define BN220_LON_SIGN 10

#define BN220_TIMEOUT 4

#include <Arduino.h>
#include "device.h"

class Bn220Interface : public AbstractDevice
{
private:
    const uint8_t nmeaData[5][9][2] = {{{0, 0}},
                                       {{1, BN220_TIME}, {2, BN220_LAT}, {3, BN220_LAT_SIGN}, {4, BN220_LON}, {5, BN220_LON_SIGN}, {7, BN220_SAT}, {9, BN220_ALT}, {0, 0}},                  // GGA
                                       {{1, BN220_LAT}, {2, BN220_LAT_SIGN}, {3, BN220_LON}, {4, BN220_LON_SIGN}, {5, BN220_TIME}, {0, 0}},                                                  // GLL
                                       {{1, BN220_TIME}, {3, BN220_LAT}, {4, BN220_LAT_SIGN}, {5, BN220_LON}, {6, BN220_LON_SIGN}, {7, BN220_SPD}, {8, BN220_COG}, {9, BN220_DATE}, {0, 0}}, // RMC
                                       {{1, BN220_COG}, {7, BN220_KPH}, {0, 0}}};                                                                  // VTG
    float lat_, lon_, alt_, spd_, cog_, kph_;
    int8_t latDir_ = 1;
    int8_t lonDir_ = 1;
    float value_[9] = {0};
    uint32_t time_, date_;
    uint8_t contIndex_ = 0, contBuff_ = 0, sat_, nmeaCmd_ = 255;
    char buffer_[20] = {};
    HardwareSerial &serial_;
    void update();
    void parser(uint8_t type, char *data);

public:
    Bn220Interface(HardwareSerial &serial);
    void begin();
    float read(uint8_t index);
};

#endif