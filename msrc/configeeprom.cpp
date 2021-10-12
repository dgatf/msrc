#include "configeeprom.h"

ConfigEeprom::ConfigEeprom() {}

Config ConfigEeprom::readConfig()
{
    Config config;
    uint32_t chk;
#if defined(FORCE_EEPROM_WRITE)
    EEPROM.put(0, (uint32_t)0);
#endif
    EEPROM.get(0, chk);
    if (chk == EEPROM_VERSION)
    {
        EEPROM.get(4, config);
    }
    else
    {
        config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
        writeConfig(config);
    }
#ifdef DEBUG_EEPROM_READ
    DEBUG_PRINT("RC");
    DEBUG_PRINT("Pr:");
    DEBUG_PRINT(config.protocol);
    DEBUG_PRINT("As:");
    DEBUG_PRINT(config.airspeed);
    DEBUG_PRINT("Gp:");
    DEBUG_PRINT(config.gps);
    DEBUG_PRINT("V1:");
    DEBUG_PRINT(config.voltage1);
    DEBUG_PRINT("V2:");
    DEBUG_PRINT(config.voltage2);
    DEBUG_PRINT("Cu:");
    DEBUG_PRINT(config.current);
    DEBUG_PRINT("T1:");
    DEBUG_PRINT(config.ntc1);
    DEBUG_PRINT("T2:");
    DEBUG_PRINT(config.ntc2);
    DEBUG_PRINT("Po:");
    DEBUG_PRINT(config.pwmOut);
    DEBUG_PRINT("rR:");
    DEBUG_PRINT(config.refresh.rpm);
    DEBUG_PRINT("rV:");
    DEBUG_PRINT(config.refresh.volt);
    DEBUG_PRINT("rC:");
    DEBUG_PRINT(config.refresh.curr);
    DEBUG_PRINT("rT:");
    DEBUG_PRINT(config.refresh.temp);
    DEBUG_PRINT("aR:");
    DEBUG_PRINT(config.average.rpm);
    DEBUG_PRINT("aV:");
    DEBUG_PRINT(config.average.volt);
    DEBUG_PRINT("aC:");
    DEBUG_PRINT(config.average.curr);
    DEBUG_PRINT("aT:");
    DEBUG_PRINT(config.average.temp);
    DEBUG_PRINT("I1:");
    DEBUG_PRINT(config.deviceI2C1Type);
    DEBUG_PRINT("A1:");
    DEBUG_PRINT(config.deviceI2C1Address);
    DEBUG_PRINT("I2:");
    DEBUG_PRINT(config.deviceI2C2Type);
    DEBUG_PRINT("A2:");
    DEBUG_PRINT(config.deviceI2C2Address);
#endif
    return config;
}

void ConfigEeprom::writeConfig(Config &config)
{
    EEPROM.put(0, (uint32_t)EEPROM_VERSION);
    EEPROM.put(4, config);
#ifdef DEBUG_EEPROM_WRITE
    DEBUG_PRINT("WC");
    DEBUG_PRINT("Pr:");
    DEBUG_PRINT(config.protocol);
    DEBUG_PRINT("As:");
    DEBUG_PRINT(config.airspeed);
    DEBUG_PRINT("Gp:");
    DEBUG_PRINT(config.gps);
    DEBUG_PRINT("V1:");
    DEBUG_PRINT(config.voltage1);
    DEBUG_PRINT("V2:");
    DEBUG_PRINT(config.voltage2);
    DEBUG_PRINT("Cu:");
    DEBUG_PRINT(config.current);
    DEBUG_PRINT("T1:");
    DEBUG_PRINT(config.ntc1);
    DEBUG_PRINT("T2:");
    DEBUG_PRINT(config.ntc2);
    DEBUG_PRINT("Po:");
    DEBUG_PRINT(config.pwmOut);
    DEBUG_PRINT("rR:");
    DEBUG_PRINT(config.refresh.rpm);
    DEBUG_PRINT("rV:");
    DEBUG_PRINT(config.refresh.volt);
    DEBUG_PRINT("rC:");
    DEBUG_PRINT(config.refresh.curr);
    DEBUG_PRINT("rT:");
    DEBUG_PRINT(config.refresh.temp);
    DEBUG_PRINT("aR:");
    DEBUG_PRINT(config.average.rpm);
    DEBUG_PRINT("aV:");
    DEBUG_PRINT(config.average.volt);
    DEBUG_PRINT("aC:");
    DEBUG_PRINT(config.average.curr);
    DEBUG_PRINT("aT:");
    DEBUG_PRINT(config.average.temp);
    DEBUG_PRINT("I1:");
    DEBUG_PRINT(config.deviceI2C1Type);
    DEBUG_PRINT("A1:");
    DEBUG_PRINT(config.deviceI2C1Address);
    DEBUG_PRINT("I2:");
    DEBUG_PRINT(config.deviceI2C2Type);
    DEBUG_PRINT("A2:");
    DEBUG_PRINT(config.deviceI2C2Address);
#endif
}
