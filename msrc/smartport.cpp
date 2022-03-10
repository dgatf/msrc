#include "smartport.h"

Smartport::Smartport(AbstractSerial &serial) : serial_(serial)
{
}

Smartport::~Smartport()
{
    deleteSensors();
}

void Smartport::begin()
{
    serial_.begin(57600, SERIAL__8N1_RXINV_TXINV | SERIAL__HALF_DUP);
    serial_.setTimeout(SMARTPORT_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
#if defined(CONFIG_LUA) && RX_PROTOCOL == RX_SMARTPORT
    config = readConfig();
#endif
    setConfig(config);
}

const uint8_t Smartport::sensorIdMatrix[29] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0xD, 0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B, 0x0};

uint8_t Smartport::sensorId()
{
    return sensorId_;
}

void Smartport::setSensorId(uint8_t sensorId)
{
    sensorId_ = sensorId;
}

uint8_t Smartport::idToCrc(uint8_t sensorId)
{
    if (sensorId < 1 || sensorId > 28)
    {
        return 0;
    }
    return sensorIdMatrix[sensorId - 1];
}

uint8_t Smartport::crcToId(uint8_t sensorIdCrc)
{
    uint8_t cont = 0;
    while (sensorIdCrc != sensorIdMatrix[cont] && cont < 28)
    {
        cont++;
    }
    if (cont == 28)
        return 0;
    return cont + 1;
}

void Smartport::sendByte(uint8_t c, uint16_t *crcp)
{

    if (crcp != NULL)
    {
        uint16_t crc = *crcp;
        crc += c;
        crc += crc >> 8;
        crc &= 0x00FF;
        *crcp = crc;
    }

    if (c == 0x7D || c == 0x7E)
    {
        serial_.write(0x7D);
        c ^= 0x20;
    }

    serial_.write(c);
}

void Smartport::sendData(uint16_t dataId, uint32_t val)
{
    sendData(0x10, dataId, val);
}

void Smartport::sendData(uint8_t typeId, uint16_t dataId, uint32_t val)
{
    digitalWrite(LED_BUILTIN, HIGH);
    uint16_t crc = 0;
    uint8_t *u8p;
    // typeId
    sendByte(typeId, &crc);
    // dataId
    u8p = (uint8_t *)&dataId;
    sendByte(u8p[0], &crc);
    sendByte(u8p[1], &crc);
    // val
    u8p = (uint8_t *)&val;
    sendByte(u8p[0], &crc);
    sendByte(u8p[1], &crc);
    sendByte(u8p[2], &crc);
    sendByte(u8p[3], &crc);
    // crc
    sendByte(0xFF - (uint8_t)crc, NULL);
    digitalWrite(LED_BUILTIN, LOW);
}

void Smartport::sendVoid()
{
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    serial_.writeBytes(buf, 8);
}

void Smartport::addSensor(Sensor *newSensorP)
{
    static Sensor *prevSensorP;
    if (sensorP == NULL)
    {
        prevSensorP = newSensorP;
        newSensorP->nextP = newSensorP;
        spSensorP = newSensorP;
    }
    sensorP = newSensorP;
    sensorP->nextP = prevSensorP->nextP;
    prevSensorP->nextP = newSensorP;
    prevSensorP = newSensorP;
}

void Smartport::addPacket(uint16_t dataId, uint32_t value)
{
    addPacket(0x10, dataId, value);
}

void Smartport::addPacket(uint8_t frameId, uint16_t dataId, uint32_t value)
{
    static Packet *lastPacketP = NULL;
    Packet *newPacketP;
    newPacketP = new Packet;
    newPacketP->frameId = frameId;
    newPacketP->dataId = dataId;
    newPacketP->value = value;
    if (packetP == NULL)
        packetP = newPacketP;
    else
        lastPacketP->nextP = newPacketP;
    lastPacketP = newPacketP;
}

void Smartport::deleteSensors()
{
    if (sensorP != NULL)
    {
        Sensor *firstSensorP, *nextSensorP;
        firstSensorP = sensorP;
        uint8_t cont = 0;
        AbstractDevice *deviceP[20];
        do
        {
            nextSensorP = sensorP->nextP;
            boolean deleteDevice = true;
            for (uint8_t i = 0; i < cont; i++)
            {
                if (deviceP[i] == sensorP->deviceP_)
                {
                    deleteDevice = false;
                    break;
                }
            }
            if (deleteDevice)
            {
                deviceP[cont] = sensorP->deviceP_;
                delete sensorP->deviceP_;
                cont++;
            }
            delete sensorP;
            sensorP = nextSensorP;
        } while (sensorP != firstSensorP);
        sensorP = NULL;
    }
    /*if (packetP != NULL)
    {
        delete packetP;
        packetP = NULL;
    }*/
}

uint8_t Smartport::getCrc(uint8_t *data)
{
    uint16_t crc = 0;
    for (uint8_t i = 2; i < 9; i++)
    {
        crc += data[i];
        crc += crc >> 8;
        crc &= 0x00FF;
    }
    return 0xFF - (uint8_t)crc;
}

uint8_t Smartport::read(uint8_t &sensorId, uint8_t &frameId, uint16_t &dataId, uint32_t &value)
{
    uint8_t packet = RECEIVED_NONE;
    uint8_t lenght = serial_.availableTimeout();
    if (lenght)
    {
        uint8_t data[15];
        serial_.readBytes(data, lenght);
#ifdef DEBUG_PACKET
            DEBUG_PRINT("< ");
            for (uint8_t i = 0; i < lenght; i++)
            {
                DEBUG_PRINT_HEX(data[i]);
                DEBUG_PRINT(" ");
            }
            DEBUG_PRINTLN();
#endif 
        if (lenght == 2 && data[0] == 0x7E && data[1] == sensorId_)
        {
            packet = RECEIVED_POLL;
        }
        if (lenght >= 10)
        {
            uint8_t i;
            uint8_t delta = 0;
            for (i = 0; i < lenght; i++)
            {
                data[i] = data[i + delta];
                if (data[i] == 0x7D)
                {
                    delta++;
                    data[i] = data[i + delta] ^ 0x20;
                }
            }
            if (data[0] == 0x7E && i == 10)
            {
                uint8_t crc = getCrc(data);
                if (crc == data[9] && data[2] != 0x00 && data[2] != 0x10)
                {
                    sensorId = data[1];
                    frameId = data[2];
                    dataId = (uint16_t)data[4] << 8 | data[3];
                    value = (uint32_t)data[8] << 24 | (uint32_t)data[7] << 16 |
                            (uint16_t)data[6] << 8 | data[5];
                    packet = RECEIVED_PACKET;
                }
            }
        }
    }
    return packet;
}

void Smartport::update()
{
    uint8_t frameId;
    uint16_t dataId;
    uint32_t value;
#if defined(SIM_LUA_SEND)
    uint8_t sensorId = sensorId_;
    uint8_t packetType = RECEIVED_PACKET;
    maintenanceMode_ = true;
    frameId = 0x30;
    dataId = DATA_ID;
    value = 0;
    if (packetP != NULL)
        packetType = RECEIVED_POLL;
    else
        delay(2000);
#elif defined(SIM_LUA_RECEIVE)
    uint8_t sensorId = sensorId_;
    uint8_t packetType = RECEIVED_PACKET;
    maintenanceMode_ = true;
    frameId = 0x31;
    dataId = DATA_ID;
    static uint8_t contPacket = 0;
    if (contPacket == 0)
        value = 0xAAAA55F1;
    if (contPacket == 1)
        value = 0x3333F2;
    if (contPacket == 2)
        value = 0x043021F3;
    if (contPacket == 3)
    {
        contPacket = 0;
        delay(2000);
    }
    contPacket++;

#elif defined(SIM_RX)
    static uint16_t ts = 0;
    uint8_t packetType = RECEIVED_NONE;
    if ((uint16_t)(millis() - ts) > 120)
    {
        packetType = RECEIVED_POLL;
        ts = millis();
    }
#else
    uint8_t sensorId;
    uint8_t packetType = read(sensorId, frameId, dataId, value);
#endif
    if (packetType == RECEIVED_POLL)
    {
        if (packetP != NULL && maintenanceMode_) // if maintenance send packet
        {
            sendData(packetP->frameId, packetP->dataId, packetP->value);
#ifdef DEBUG_PACKET
            DEBUG_PRINT(">F:");
            DEBUG_PRINT_HEX(packetP->frameId);
            DEBUG_PRINT(" D:");
            DEBUG_PRINT_HEX(packetP->dataId);
            DEBUG_PRINT(" V:");
            DEBUG_PRINT_HEX(packetP->value);
            DEBUG_PRINTLN();
#endif
            Packet *oldPacketP;
            oldPacketP = packetP;
            packetP = packetP->nextP;
            delete oldPacketP;
        }
        if (sensorP != NULL && !maintenanceMode_) // else send telemetry
        {
            // loop sensors until correct timestamp or 1 sensors cycle
            Sensor *initialSensorP = spSensorP;
            while ( ((uint16_t)(millis() - spSensorP->timestamp()) <= (uint16_t)spSensorP->refresh() * 100) && spSensorP->nextP != initialSensorP )
            {
                spSensorP = spSensorP->nextP;
            }
            if ( (uint16_t)(millis() - spSensorP->timestamp()) >= (uint16_t)spSensorP->refresh() * 100 )
            {
                sendData(spSensorP->frameId(), spSensorP->dataId(), spSensorP->valueFormatted());
#ifdef DEBUG
                DEBUG_PRINT("D:");
                DEBUG_PRINT_HEX(spSensorP->dataId());
                DEBUG_PRINT(" V:");
                DEBUG_PRINT(spSensorP->valueFormatted());
                spSensorP->valueFormatted(); // toggle type if date/time or lat/lon sensor
                DEBUG_PRINT(" T:");
                DEBUG_PRINT(spSensorP->timestamp());
                DEBUG_PRINTLN();
#endif
                spSensorP->setTimestamp(millis());
                spSensorP = spSensorP->nextP;
            }
            else
            {
                sendVoid();
            }
        }
    }
    else if (packetType == RECEIVED_PACKET)
    {
#ifdef DEBUG_PACKET
        DEBUG_PRINT("<F:");
        DEBUG_PRINT_HEX(frameId);
        DEBUG_PRINT(" D:");
        DEBUG_PRINT_HEX(dataId);
        DEBUG_PRINT(" V:");
        DEBUG_PRINT_HEX(value);
        DEBUG_PRINTLN();
#endif
        processPacket(frameId, dataId, value);
    }
    // update sensor
    if (sensorP != NULL)
    {
        sensorP->update();
        sensorP = sensorP->nextP;
    }
}

void Smartport::setConfig(Config &config)
{
    deleteSensors();
    setSensorId(idToCrc(config.sensorId));
    if (ESC_PROTOCOL == PROTOCOL_PWM)
    {
        Sensor *sensorP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), NULL, config.refresh.rpm, esc);
        addSensor(sensorP);
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V3)
    {
        Sensor *sensorP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), NULL, config.refresh.rpm, esc);
        addSensor(sensorP);
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        Sensor *sensorP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), esc->consumptionP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->voltageP(), esc->currentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->tempFetP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, esc->tempBecP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
#ifdef ESC_SIGNATURE
        sensorP = new Sensor(DIY_FIRST_ID, esc->signatureP(), 20, esc);
        addSensor(sensorP);
        sensorP = new Sensor(DIY_FIRST_ID + 1, esc->signatureP() + 1, 20, esc);
        addSensor(sensorP);
        sensorP = new Sensor(DIY_FIRST_ID + 2, esc->signatureP() + 2, 20, esc);
        addSensor(sensorP);
#endif
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        Sensor *sensorP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), esc->consumptionP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->voltageP(), esc->currentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(SBEC_POWER_FIRST_ID, esc->becVoltageP(), esc->becCurrentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID + 1, esc->rippleVoltageP(), NULL, config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->temperatureP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        Sensor *sensorP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), esc->consumptionP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->voltageP(), esc->currentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(SBEC_POWER_FIRST_ID, esc->becVoltageP(), esc->becCurrentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->tempFetP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, esc->tempBecP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        Sensor *sensorP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), esc->consumptionP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->voltageP(), esc->currentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->tempP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        Sensor *sensorP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), esc->consumptionP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->voltageP(), esc->currentP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->tempP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.gps == true)
    {
        Sensor *sensorP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        sensorP = new SensorLatLon(GPS_LONG_LATI_FIRST_ID, gps->lonP(), gps->latP(), 5, gps);
        addSensor(sensorP);
        sensorP = new Sensor(GPS_ALT_FIRST_ID, gps->altP(), 5, gps);
        addSensor(sensorP);
        sensorP = new Sensor(GPS_SPEED_FIRST_ID, gps->spdP(), 5, gps);
        addSensor(sensorP);
        sensorP = new Sensor(GPS_COURS_FIRST_ID, gps->cogP(), 5, gps);
        addSensor(sensorP);
        sensorP = new SensorDateTime(GPS_TIME_DATE_FIRST_ID, gps->timeP(), gps->dateP(), 5, gps);
        addSensor(sensorP);
        sensorP = new Sensor(VARIO_FIRST_ID + 1, gps->varioP(), 5, gps);
        addSensor(sensorP);
        sensorP = new Sensor(DIY_FIRST_ID + 3, gps->satP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensor(DIY_FIRST_ID + 4, gps->distP(), 10, gps);
        addSensor(sensorP);
    }
    if (config.airspeed == true)
    {
        Sensor *sensorP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorP = new Sensor(AIR_SPEED_FIRST_ID, pressure->valueP(), config.refresh.volt, pressure);
        addSensor(sensorP);
    }
    if (config.voltage1 == true)
    {
        Sensor *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        sensorP = new Sensor(A3_FIRST_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
    }
    if (config.voltage2 == true)
    {
        Sensor *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt), VOLTAGE2_MULTIPLIER);
        sensorP = new Sensor(A4_FIRST_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
    }
    if (config.current == true)
    {
        Sensor *sensorP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        sensorP = new Sensor(CURR_FIRST_ID, current->valueP(), config.refresh.curr, current);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_RPM_CONS_FIRST_ID + 1, NULL, current->consumptionP(), config.refresh.curr, current);
        addSensor(sensorP);
    }
    if (config.ntc1 == true)
    {
        Sensor *sensorP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorP = new Sensor(T1_FIRST_ID, ntc->valueP(), config.refresh.temp, ntc);
        addSensor(sensorP);
    }
    if (config.ntc2 == true)
    {
        Sensor *sensorP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorP = new Sensor(T2_FIRST_ID, ntc->valueP(), config.refresh.temp, ntc);
        addSensor(sensorP);
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        Sensor *sensorP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorP = new Sensor(T1_FIRST_ID + 1, bmp->temperatureP(), config.refresh.temp, bmp);
        addSensor(sensorP);
        sensorP = new Sensor(ALT_FIRST_ID, bmp->altitudeP(), CONFIG_AVERAGING_ELEMENTS_DEF, bmp);
        addSensor(sensorP);
        sensorP = new Sensor(VARIO_FIRST_ID, bmp->varioP(), 5, bmp);
        addSensor(sensorP);
    }
        if (config.deviceI2C1Type == I2C_MS5611)
    {
        Sensor *sensorP;
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorP = new Sensor(T1_FIRST_ID + 1, bmp->temperatureP(), config.refresh.temp, bmp);
        addSensor(sensorP);
        sensorP = new Sensor(ALT_FIRST_ID, bmp->altitudeP(), CONFIG_AVERAGING_ELEMENTS_DEF, bmp);
        addSensor(sensorP);
        sensorP = new Sensor(VARIO_FIRST_ID, bmp->varioP(), 5, bmp);
        addSensor(sensorP);
    }
}

void Smartport::processPacket(uint8_t frameId, uint16_t dataId, uint32_t value)
{
    // maintenance mode on
    if (frameId == 0x21 && dataId == 0xFFFF && value == 0x80)
    {
#ifdef DEBUG
        DEBUG_PRINT("M+");
        DEBUG_PRINTLN();
#endif
        maintenanceMode_ = true;
        return;
    }

    // maintenance mode off
    if (frameId == 0x20 && dataId == 0xFFFF && value == 0x80)
    {
#ifdef DEBUG
        DEBUG_PRINT("M-");
        DEBUG_PRINTLN();
#endif
        maintenanceMode_ = false;
        return;
    }

    // send sensorId
    if (maintenanceMode_ && frameId == 0x30 && dataId == dataId_ && value == 0x01)
    {
#ifdef DEBUG
        DEBUG_PRINT("S>");
        DEBUG_PRINT(crcToId(sensorId_));
        DEBUG_PRINTLN();
#endif
        addPacket(0x32, dataId_, (crcToId(sensorId_) - 1) << 8 | 0x01);
        return;
    }

    // change sensorId
    if (maintenanceMode_ && frameId == 0x31 && dataId == dataId_ && (uint8_t)value == 0x01)
    {
        sensorId_ = idToCrc((value >> 8) + 1);
        Config config = readConfig();
        config.sensorId = sensorId_;
        writeConfig(config);
#ifdef DEBUG
        DEBUG_PRINT("S<");
        DEBUG_PRINT(crcToId(sensorId_));
        DEBUG_PRINTLN();
#endif
        return;
    }

    // send config
    if (maintenanceMode_ && frameId == 0x30 && dataId == DATA_ID && value == 0)
    {
        uint32_t value = 0;
        Config config = readConfig();
        // packet 1
        value = 0xF1;
        value |= (uint32_t)VERSION_PATCH << 8;
        value |= (uint32_t)VERSION_MINOR << 16;
        value |= (uint32_t)VERSION_MAJOR << 24;
        addPacket(0x32, DATA_ID, value);
        // packet 2, 3 & 4
        for (uint8_t i = 0; i < 3; i++)
        {
            value = 0xF2 + i;
            memcpy((uint8_t *)&value + 1, (uint8_t *)&config + 3 * i, 3);
            addPacket(0x32, DATA_ID, value);
        }
        return;
    }

    // receive config
    if (maintenanceMode_ && frameId == 0x31 && dataId == DATA_ID && ((uint8_t)(value) == 0xF1 || (uint8_t)(value) == 0xF2 || (uint8_t)(value) == 0xF3))
    {
        Config config = readConfig();
        uint8_t i = value - 0xF1;
        memcpy((uint8_t *)&config + 3 * i, (uint8_t *)&value + 1, 3);
        writeConfig(config);
        if ((uint8_t)value == 0xF3)
        {
            setConfig(config);
            addPacket(0x32, DATA_ID, 0xFF);
            PwmOut pwmOut;
            if (config.pwmOut)
                pwmOut.enable();
            else
                pwmOut.disable();
        }
    }
}