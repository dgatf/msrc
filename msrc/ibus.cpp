#include "ibus.h"

Ibus::Ibus(Stream &serial) : serial_(serial)
{
}

Ibus::~Ibus()
{
    deleteSensors();
}

void Ibus::begin()
{
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
    DEBUG_SERIAL.print(c, HEX);
    DEBUG_SERIAL.print(" ");
#endif
}

void Ibus::sendData(uint8_t command, uint8_t address)
{
    digitalWrite(LED_BUILTIN, HIGH);

    uint8_t *u8P;
    uint16_t crc = 0;
    uint32_t value;

    // get value
    uint8_t lenght = 0;
    switch (command)
    {
    case IBUS_COMMAND_DISCOVER:
        lenght = 0;
        break;
    case IBUS_COMMAND_TYPE:
        value = 0x02 << 8 | sensorIbusP[address]->dataId();
        u8P = (uint8_t *)&value;
        lenght = 2;
        break;
    case IBUS_COMMAND_MEASURE:
        value = sensorIbusP[address]->valueFormatted();
        u8P = (uint8_t *)&value;
        if (sensorIbusP[address]->type() == IBUS_TYPE_S16 || sensorIbusP[address]->type() == IBUS_TYPE_U16)
            lenght = 2;
        else
            lenght = 4;
        break;
    }
#ifdef DEBUG
    DEBUG_SERIAL.print("> ");
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
    DEBUG_SERIAL.println();
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

void Ibus::deleteSensors()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (sensorIbusP[i] != NULL)
            delete sensorIbusP[i];
    }
}

bool Ibus::checkCrc(uint8_t *data)
{
    uint16_t crc = 0;
    uint8_t lenght = data[0];
    for (uint8_t i = 0; i < lenght - 2; i++)
        crc += data[i];
    if (crc == (uint16_t)data[lenght - 1] << 8 || data[lenght])
        return true;
    return false;
}

uint8_t Ibus::read(uint8_t &command, uint8_t &address)
{
    static uint16_t serialTs = 0;
    static uint8_t serialCount = 0;
    uint8_t packet = IBUS_RECEIVED_NONE;
    if (serial_.available() > serialCount)
    {
        serialCount = serial_.available();
        serialTs = micros();
    }
    if (((uint16_t)micros() - serialTs > IBUS_TIMEOUT) && serialCount > 0)
    {
        if (serialCount == IBUS_PACKET_LENGHT)
        {
            uint8_t data[IBUS_PACKET_LENGHT];
            serial_.readBytes(data, IBUS_PACKET_LENGHT);
            if (data[0] == 4)
            {
#ifdef DEBUG
                DEBUG_SERIAL.print("< ");
                for (uint8_t i = 0; i < data[0]; i++)
                {
                    DEBUG_SERIAL.print(data[i], HEX);
                    DEBUG_SERIAL.print(" ");
                }
#endif
                if (checkCrc(data))
                {
#ifdef DEBUG
                    DEBUG_SERIAL.println("CRC OK");
#endif
                    command = data[1] >> 4;
                    address = data[1] & 0x0F;
                    if (command == IBUS_COMMAND_DISCOVER || command == IBUS_COMMAND_TYPE || IBUS_COMMAND_MEASURE)
                        packet = IBUS_RECEIVED_POLL;
                }
            }
        }
        while (serial_.available())
            serial_.read();
        serialCount = 0;
    }
    return packet;
}

void Ibus::update()
{
#if defined(SIM_RX)
    static uint8_t command = IBUS_COMMAND_DISCOVER;
    static uint8_t address = 0;
    static uint16_t ts = 0;
    uint8_t packetType = IBUS_RECEIVED_NONE;
    if ((uint16_t)millis() - ts > 100)
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
    deleteSensors();
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
        ESC_SERIAL.begin(19200);
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        SensorIbus *sensorIbusP;
        EscHW4 *esc;
        ESC_SERIAL.begin(19200);
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), config.protocol - PROTOCOL_HW_V4_LV);
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
        //sensorIbusP = new SensorIbus(TEMP2_ID, esc->tempBecP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
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
        //sensorIbusP = new SensorIbus(SBEC_POWER_FIRST_ID, esc->becVoltageP(), esc->becCurrentP(), esc);
        //addSensor(sensorIbusP);
        //sensorIbusP = new SensorIbus(ESC_POWER_FIRST_ID + 1, esc->rippleVoltageP(), NULL, config.refresh.volt, esc);
        //addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->temperatureP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        SensorIbus *sensorIbusP;
        EscKontronik *esc;
        ESC_SERIAL.begin(115200, SERIAL_8E1);
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorIbusP = new SensorIbus(AFHDS2A_ID_MOT, IBUS_TYPE_U16, esc->rpmP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, esc->voltageP(), esc);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, esc->currentP(), esc);
        addSensor(sensorIbusP);
        //sensorIbusP = new SensorIbus(SBEC_POWER_FIRST_ID, esc->becVoltageP(), esc->becCurrentP(), esc);
        //addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, esc->tempFetP(), esc);
        addSensor(sensorIbusP);
        //sensorIbusP = new SensorIbus(TEMP2_ID, esc->tempBecP(), esc);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, esc->cellVoltageP(), esc);
        addSensor(sensorIbusP);
    }
    if (config.gps == true)
    {
        SensorIbus *sensorIbusP;
        Bn220 *gps;
        GPS_SERIAL.begin(GPS_BAUD_RATE);
        GPS_SERIAL.setTimeout(BN220_TIMEOUT);
        gps = new Bn220(GPS_SERIAL);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_LAT, IBUS_TYPE_S32, gps->lonP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_LON, IBUS_TYPE_S32, gps->latP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_GPS_ALT, IBUS_TYPE_S32, gps->altP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_SPE, IBUS_TYPE_U16, gps->spdP(), gps);
        addSensor(sensorIbusP);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_COG, IBUS_TYPE_U16, gps->cogP(), gps);
        addSensor(sensorIbusP);
    }
    if (config.airspeed == true)
    {
        SensorIbus *sensorIbusP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorIbusP = new SensorIbus(AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, pressure->valueP(), pressure);
        addSensor(sensorIbusP);
    }
    if (config.voltage1 == true)
    {
        SensorIbus *sensorIbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt));
        sensorIbusP = new SensorIbus(AFHDS2A_ID_EXTV, IBUS_TYPE_U16, voltage->valueP(), voltage);
        addSensor(sensorIbusP);
    }
    /*if (config.voltage2 == true)
    {
        SensorIbus *sensorIbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt));
        sensorIbusP = new SensorIbus(A4_FIRST_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorIbusP);
    }*/
    if (config.current == true)
    {
        SensorIbus *sensorIbusP;
        Voltage *current;
        current = new Voltage(PIN_CURRENT, ALPHA(config.average.curr));
        sensorIbusP = new SensorIbus(AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, current->valueP(), current);
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
    /*if (config.ntc2 == true)
    {
        SensorIbus *sensorIbusP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorIbusP = new SensorIbus(TEMP2_ID, ntc->valueP(), ntc);
        addSensor(sensorIbusP);
    }*/
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        SensorIbus *sensorIbusP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), 10);
        bmp->begin();
        //sensorIbusP = new SensorIbus(TEMP1_ID, bmp->temperatureP(), bmp);
        sensorIbusP = new SensorIbus(AFHDS2A_ID_ALT_FLYSKY, IBUS_TYPE_U16, bmp->altitudeP(), bmp);
        addSensor(sensorIbusP);
    }
}