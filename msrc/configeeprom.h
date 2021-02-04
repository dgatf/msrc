#ifndef CONFIGEEPROM_H
#define CONFIGEEPROM_H

#define EEPROM_VERSION 1

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

struct Level
{
    uint8_t rpm : 4;  // rpm averaging elements
    uint8_t volt : 4; // voltage averaging elements
    uint8_t curr : 4; // current averaging elements
    uint8_t temp : 4; // temperature averaging elements
};

struct Config
{
    // 24
    bool airspeed : 1; // enable/disable pressure analog reading
    bool gps : 1;      // enable/disable serial gps (not feasible with esc serial)
    bool voltage1 : 1; // enable/disable voltage1 analog reading
    bool voltage2 : 1; // enable/disable voltage2 analog reading
    bool current : 1;  // enable/disable current analog reading
    bool ntc1 : 1;     // enable/disable ntc1 analog reading
    bool ntc2 : 1;     // enable/disable ntc2 analog reading
    bool pwmOut : 1;   // enable/disable pwm out generation (governor)
    Level refresh;     // max refresh rate 15 (1.5s)

    // 24
    Level average;
    uint8_t protocol; // esc protocol

    // 24
    uint8_t deviceI2C1Type : 4;
    uint8_t deviceI2C1Address : 8;
    uint8_t deviceI2C2Type : 4;    // unused
    uint8_t deviceI2C2Address : 8; // unused

    uint8_t sensorId;
};

class ConfigEeprom
{
public:
    ConfigEeprom();
    Config readConfig();
    void writeConfig(Config &config);
};

#endif