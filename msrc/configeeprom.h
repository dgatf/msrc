#ifndef CONFIGEEPROM_H
#define CONFIGEEPROM_H

#include <Arduino.h>
#include <EEPROM.h>

struct Level
{
    uint8_t rpm:4; // = CONFIG_ALPHA_RPM;  // rpm averaging elements (alpha * 100)
    uint8_t volt:4; // = CONFIG_ALPHA_VOLT; // voltage averaging elements (alpha * 100)
    uint8_t curr:4; // = CONFIG_ALPHA_CURR; // current averaging elements (alpha * 100)
    uint8_t temp:4; // = CONFIG_ALPHA_TEMP; // temperature averaging elements (alpha * 100)
};

struct DeviceI2C
{
    uint8_t type:4; //= 0;
    uint8_t address; //= 0;
};

struct Config
{
    // 24
    bool airspeed:1; //= CONFIG_AIRSPEED;            // enable/disable pressure analog reading
    bool gps:1; // = CONFIG_GPS;                 // enable/disable serial gps (not feasible with esc serial)
    bool voltage1:1; // = CONFIG_VOLTAGE1;            // enable/disable voltage1 analog reading
    bool voltage2:1; // = CONFIG_VOLTAGE2;            // enable/disable voltage2 analog reading
    bool current:1; // = CONFIG_CURRENT;             // enable/disable current analog reading
    bool ntc1:1; // = CONFIG_NTC1;                // enable/disable ntc1 analog reading
    bool ntc2:1; // = CONFIG_NTC2;                // enable/disable ntc2 analog reading
    bool pwmOut:1; // = CONFIG_PWMOUT;              // enable/disable pwm out generation (governor)
    Level refresh; // max refresh rate 15 (1.5s)

    // 24
    Level average; // 16 equivalent EMA to SMA elements (alpha=2/(N+1))
    uint8_t protocol; // = CONFIG_ESC_PROTOCOL; // esc protocol

    // 24
    DeviceI2C deviceI2C[2]; // 12*2

    uint8_t sensorId; //= SENSOR_ID;
};

class ConfigEeprom
{
public:
    ConfigEeprom();
    Config readConfig();
    void writeConfig(Config &config);
};

#endif