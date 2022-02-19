#ifndef BN220_H
#define BN220_H

#define BN220_UNK 0
#define BN220_GGA 1
#define BN220_RMC 2
#define BN220_GSA 3

#define BN220_VTG 4
#define BN220_GLL 5


#define BN220_LON 1
#define BN220_ALT 2
#define BN220_SPD 3
#define BN220_COG 4
#define BN220_FIX 5
#define BN220_SAT 6
#define BN220_DATE 7
#define BN220_TIME 8
#define BN220_LAT_SIGN 9
#define BN220_LON_SIGN 10
#define BN220_HDOP 11
#define BN220_END 12
#define BN220_LAT 13

#define BN220_TIMEOUT 2000

#include <Arduino.h>
#include "device.h"

class Bn220 : public AbstractDevice, Vario
{
private:
    const uint8_t gga[10] = {0, 0, 0, 0, 0, 0, BN220_SAT, BN220_HDOP, BN220_ALT, BN220_END}; // GGA
    const uint8_t rmc[10] = {BN220_TIME, 0, BN220_LAT, BN220_LAT_SIGN, BN220_LON, BN220_LON_SIGN, BN220_SPD, BN220_COG, BN220_DATE, BN220_END}; // RMC
    float lat_ = 0, lon_ = 0, alt_ = 0, spd_ = 0, cog_ = 0, hdop_ = 0, sat_ = 0, vario_ = 0, dist_ = 10000;
    int8_t latDir_ = 1, lonDir_ = 1;
    float time_ = 0, date_ = 0;
    uint8_t nmeaCmd_ = 0;
    char buffer_[20] = {};
    AbstractSerial &serial_;
    uint32_t baud_;
    uint8_t contIndex = 0;
    void parser(uint8_t type, char *data);
    void processField(const uint8_t *command);
    float degreesToRadians(float degrees);
    float calcDistanceToHome(float lat, float lon, uint16_t intervalMin); // distance in meters

public:
    Bn220(AbstractSerial &serial, uint32_t baud);
    void begin();
    void update();
    float *latP();
    float *lonP();
    float *altP();
    float *spdP();
    float *cogP();
    float *satP();
    float *dateP();
    float *timeP();
    float *hdopP();
    float *varioP();
    float *distP();
};

#endif