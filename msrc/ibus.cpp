#include "ibus.h"

Ibus::Ibus(AbstractSerial &serial) : serial_(serial)
{
}

Ibus::~Ibus()
{
}

void Ibus::begin()
{
    serial_.begin(115200, SERIAL__8N1 | SERIAL__HALF_DUP);
    serial_.setTimeout(IBUS_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Ibus::sendByte(uint8_t c, uint16_t *crcP)
{
    if (crcP != NULL)
    {
        uint16_t crc = *crcP;
        crc += c;
        *crcP = crc;
    }
    serial_.write(c);
#ifdef DEBUG
    DEBUG_PRINT_HEX(c);
    DEBUG_PRINT(" ");
#endif
}

void Ibus::sendData(uint8_t command, uint8_t address)
{
    digitalWrite(LED_BUILTIN, HIGH);

    uint8_t *u8P = NULL;
    uint16_t crc = 0;
    uint16_t type;
    uint8_t lenght = 0;

    switch(sensorIbusP[address]->type())
    {
    case IBUS_TYPE_S16:
    case IBUS_TYPE_U16:
        lenght = 2;
        break;
    case IBUS_TYPE_S32:
        lenght = 4;
        break;
    case IBUS_TYPE_GPS:
        lenght = 14;
        break;
    } 
    switch (command)
    {
    case IBUS_COMMAND_DISCOVER:
        lenght = 0;
        break;
    case IBUS_COMMAND_TYPE:
        type = lenght << 8 | sensorIbusP[address]->dataId();
        u8P = (uint8_t *)&type;
        lenght = 2;
        break;
    case IBUS_COMMAND_MEASURE:
        u8P = sensorIbusP[address]->valueFormatted();
        break;
    }
#ifdef DEBUG
    DEBUG_PRINT("> ");
#endif
    // lenght
    sendByte(4 + lenght, &crc);

    // command & address
    sendByte(command << 4 | address, &crc);

    //value
    for (uint8_t i = 0; i < lenght; i++)
        sendByte(u8P[i], &crc);

    // crc
    crc = 0xFFFF - crc;
    u8P = (uint8_t *)&crc;
    sendByte(u8P[0], NULL);
    sendByte(u8P[1], NULL);

    digitalWrite(LED_BUILTIN, LOW);
#ifdef DEBUG
    DEBUG_PRINTLN();
#endif
}

void Ibus::addSensor(SensorIbus *newSensorIbusP)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (sensorIbusP[i] == NULL && sensorMask & 1 << i)
        {
            sensorIbusP[i] = newSensorIbusP;
            return;
        }
    }
}

bool Ibus::checkCrc(uint8_t *data)
{
    uint16_t crc = 0xFFFF;
    uint8_t lenght = data[0];
    for (uint8_t i = 0; i < lenght - 2; i++)
        crc -= data[i];
    if ( crc == (uint16_t)data[lenght - 1] << 8 || data[lenght] )
        return true;
    return false;
}

uint8_t Ibus::read(uint8_t &command, uint8_t &address)
{
    if (serial_.availableTimeout() == IBUS_PACKET_LENGHT)
    {
        uint8_t data[IBUS_PACKET_LENGHT];
        serial_.readBytes(data, IBUS_PACKET_LENGHT);
        if (data[0] == IBUS_PACKET_LENGHT)
        {
#ifdef DEBUG
            DEBUG_PRINT("< ");
            for (uint8_t i = 0; i < data[0]; i++)
            {
                DEBUG_PRINT_HEX(data[i]);
                DEBUG_PRINT(" ");
            }
#endif
            if (checkCrc(data))
            {
#ifdef DEBUG
                DEBUG_PRINT("+");
                DEBUG_PRINTLN();
#endif
                command = data[1] >> 4;
                address = data[1] & 0x0F;
                if (command == IBUS_COMMAND_DISCOVER || command == IBUS_COMMAND_TYPE || IBUS_COMMAND_MEASURE)
                    return IBUS_RECEIVED_POLL;
            }
#ifdef DEBUG
            DEBUG_PRINTLN();
#endif
        }
    }
    return IBUS_RECEIVED_NONE;
}

void Ibus::update()
{
#if defined(SIM_RX)
    static uint8_t command = IBUS_COMMAND_DISCOVER;
    static uint8_t address = 0;
    static uint16_t ts = 0;
    uint8_t packetType = IBUS_RECEIVED_NONE;
    if ((uint16_t)(millis() - ts) > 100)
    {
        packetType = IBUS_RECEIVED_POLL;
        address++;
        ts = millis();
        if (address == 16)
        {
            address = 0;
            if (command < IBUS_COMMAND_MEASURE)
                command++;
        }
    }
#else
    uint8_t command;
    uint8_t address;
    uint8_t packetType = read(command, address);
#endif
    if (packetType == IBUS_RECEIVED_POLL && sensorIbusP[address])
        sendData(command, address);

    // update sensor
    static uint8_t cont = 0;
    if (sensorIbusP[cont])
        sensorIbusP[cont]->update();
    cont++;
    if (cont == 16)
        cont = 0;
}

void Ibus::setConfig(Config &config)
{
     /*
     - Sensor at address 0x00 is reserved
     - Sensor at address 0x01 is recerved in some receivers types. But the poll at address 0x01, if present, has to be answered (so there is a dummy sensor at address 0x01), otherwise the sensor poll scan is stopped from receiver
     - TODO: dinamically set the sensor address to allocate an additional sensor in receivers with only one sensor masked
    */
    SensorIbus *sensorIbusP;
    sensorIbusP = new SensorIbus(AFHDS2A_ID_END, 0, NULL, NULL);
    addSensor(sensorIbusP);
    if (config.protocol == PROTOCOL_PWM)
    {
        SensorIbus *sensorIbusP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        SensorIbus *sensorIbusP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_HW_V4)
    {
        SensorIbus *sensorIbusP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->voltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->currentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempFetP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempBecP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_FUEL, IBUS_TYPE_U16, esc->consumptionP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_CASTLE)
    {
        SensorIbus *sensorIbusP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->voltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->currentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->becVoltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->becCurrentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->rippleVoltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->temperatureP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_FUEL, IBUS_TYPE_U16, esc->consumptionP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        SensorIbus *sensorIbusP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->voltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->currentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->becVoltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->becCurrentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempFetP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempBecP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_FUEL, IBUS_TYPE_U16, esc->consumptionP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_APD_F)
    {
        SensorIbus *sensorIbusP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->voltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->currentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_FUEL, IBUS_TYPE_U16, esc->consumptionP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_APD_HV)
    {
        SensorIbus *sensorIbusP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->voltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->currentP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_FUEL, IBUS_TYPE_U16, esc->consumptionP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.gps == true)
    {
        SensorIbus *sensorIbusP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_STATUS, IBUS_TYPE_U16, gps->satP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_LAT, IBUS_TYPE_S32, gps->latP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_LON, IBUS_TYPE_S32, gps->lonP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_ALT, IBUS_TYPE_S32, gps->altP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_SPE, IBUS_TYPE_U16, gps->spdP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_COG, IBUS_TYPE_U16, gps->cogP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, gps->varioP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_DIST, IBUS_TYPE_U16, gps->distP(), gps);
        addSensor(sensorIbusP);
#ifdef IBUS_GPS_ALTERNATIVE_LAT_LON
        sensorIbusP = new SensorIbus(AFHDS2A_ID_S84, IBUS_TYPE_S32, gps->latP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_S85, IBUS_TYPE_S32, gps->lonP(), gps);
        addSensor(sensorIbusP);
#endif
    }
    if (config.airspeed == true)
    {
        SensorIbus *sensorIbusP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorIbusP = new SensorIbus(AFHDS2A_ID_SPE, IBUS_TYPE_S16, pressure->valueP(), pressure);
        addSensor(sensorIbusP);
    }
    if (config.voltage1 == true)
    {
        SensorIbus *sensorIbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, voltage->valueP(), voltage);
        addSensor(sensorIbusP);
    }
    if (config.voltage2 == true)
    {
        SensorIbus *sensorIbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt), VOLTAGE2_MULTIPLIER);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, voltage->valueP(), voltage);
        addSensor(sensorIbusP);
    }
    if (config.current == true)
    {
        SensorIbus *sensorIbusP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, current->valueP(), current);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_FUEL, IBUS_TYPE_U16, current->consumptionP(), current);
        addSensor(sensorIbusP);
    }
    if (config.ntc1 == true)
    {
        SensorIbus *sensorIbusP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, ntc->valueP(), ntc);
        addSensor(sensorIbusP);
    }
    if (config.ntc2 == true)
    {
        SensorIbus *sensorIbusP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, ntc->valueP(), ntc);
        addSensor(sensorIbusP);
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        SensorIbus *sensorIbusP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, bmp->temperatureP(), bmp);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_ALT, IBUS_TYPE_S32, bmp->altitudeP(), bmp);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, bmp->varioP(), bmp);
        addSensor(sensorIbusP);
    }
    if (config.deviceI2C1Type == I2C_MS5611)
    {
        SensorIbus *sensorIbusP;
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, bmp->temperatureP(), bmp);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_ALT, IBUS_TYPE_S32, bmp->altitudeP(), bmp);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, bmp->varioP(), bmp);
        addSensor(sensorIbusP);
    }
}