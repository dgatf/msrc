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
        config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, 0, 0, 0, 0, SENSOR_ID};
        writeConfig(config);
    }
#if defined(DEBUG_EEPROM_READ)
    DEBUG_SERIAL.println("RC");
    DEBUG_SERIAL.print("Pr:");
    DEBUG_SERIAL.println(config.protocol);
    DEBUG_SERIAL.print("As:");
    DEBUG_SERIAL.println(config.airspeed);
    DEBUG_SERIAL.print("Gp:");
    DEBUG_SERIAL.println(config.gps);
    DEBUG_SERIAL.print("V1:");
    DEBUG_SERIAL.println(config.voltage1);
    DEBUG_SERIAL.print("V2:");
    DEBUG_SERIAL.println(config.voltage2);
    DEBUG_SERIAL.print("Cu:");
    DEBUG_SERIAL.println(config.current);
    DEBUG_SERIAL.print("T1:");
    DEBUG_SERIAL.println(config.ntc1);
    DEBUG_SERIAL.print("T2:");
    DEBUG_SERIAL.println(config.ntc2);
    DEBUG_SERIAL.print("Po:");
    DEBUG_SERIAL.println(config.pwmOut);
    DEBUG_SERIAL.print("rR:");
    DEBUG_SERIAL.println(config.refresh.rpm);
    DEBUG_SERIAL.print("rV:");
    DEBUG_SERIAL.println(config.refresh.volt);
    DEBUG_SERIAL.print("rC:");
    DEBUG_SERIAL.println(config.refresh.curr);
    DEBUG_SERIAL.print("rT:");
    DEBUG_SERIAL.println(config.refresh.temp);
    DEBUG_SERIAL.print("aR:");
    DEBUG_SERIAL.println(config.alpha.rpm);
    DEBUG_SERIAL.print("aV:");
    DEBUG_SERIAL.println(config.alpha.volt);
    DEBUG_SERIAL.print("aC:");
    DEBUG_SERIAL.println(config.alpha.curr);
    DEBUG_SERIAL.print("aT:");
    DEBUG_SERIAL.println(config.alpha.temp);
    DEBUG_SERIAL.print("I1:");
    DEBUG_SERIAL.println(config.deviceI2C1Type);
    DEBUG_SERIAL.print("A1:");
    DEBUG_SERIAL.println(config.deviceI2C1Address);
    DEBUG_SERIAL.print("I2:");
    DEBUG_SERIAL.println(config.deviceI2C2Type);
    DEBUG_SERIAL.print("A2:");
    DEBUG_SERIAL.println(config.deviceI2C2Address);
#endif
    return config;
}

void ConfigEeprom::writeConfig(Config &config)
{
    EEPROM.put(0, (uint32_t)EEPROM_VERSION);
    EEPROM.put(4, config);
#if defined(DEBUG_EEPROM_WRITE)
    DEBUG_SERIAL.println("WC");
    DEBUG_SERIAL.print("Pr:");
    DEBUG_SERIAL.println(config.protocol);
    DEBUG_SERIAL.print("As:");
    DEBUG_SERIAL.println(config.airspeed);
    DEBUG_SERIAL.print("Gp:");
    DEBUG_SERIAL.println(config.gps);
    DEBUG_SERIAL.print("V1:");
    DEBUG_SERIAL.println(config.voltage1);
    DEBUG_SERIAL.print("V2:");
    DEBUG_SERIAL.println(config.voltage2);
    DEBUG_SERIAL.print("Cu:");
    DEBUG_SERIAL.println(config.current);
    DEBUG_SERIAL.print("T1:");
    DEBUG_SERIAL.println(config.ntc1);
    DEBUG_SERIAL.print("T2:");
    DEBUG_SERIAL.println(config.ntc2);
    DEBUG_SERIAL.print("Po:");
    DEBUG_SERIAL.println(config.pwmOut);
    DEBUG_SERIAL.print("rR:");
    DEBUG_SERIAL.println(config.refresh.rpm);
    DEBUG_SERIAL.print("rV:");
    DEBUG_SERIAL.println(config.refresh.volt);
    DEBUG_SERIAL.print("rC:");
    DEBUG_SERIAL.println(config.refresh.curr);
    DEBUG_SERIAL.print("rT:");
    DEBUG_SERIAL.println(config.refresh.temp);
    DEBUG_SERIAL.print("aR:");
    DEBUG_SERIAL.println(config.average.rpm);
    DEBUG_SERIAL.print("aV:");
    DEBUG_SERIAL.println(config.average.volt);
    DEBUG_SERIAL.print("aC:");
    DEBUG_SERIAL.println(config.average.curr);
    DEBUG_SERIAL.print("aT:");
    DEBUG_SERIAL.println(config.average.temp);
    DEBUG_SERIAL.print("I1:");
    DEBUG_SERIAL.println(config.deviceI2C1Type);
    DEBUG_SERIAL.print("A1:");
    DEBUG_SERIAL.println(config.deviceI2C1Address);
    DEBUG_SERIAL.print("I2:");
    DEBUG_SERIAL.println(config.deviceI2C2Type);
    DEBUG_SERIAL.print("A2:");
    DEBUG_SERIAL.println(config.deviceI2C2Address);
#endif
}
