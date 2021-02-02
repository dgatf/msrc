#include "smartport.h"

Smartport::Smartport(Stream &serial) : serial_(serial)
{
}

Smartport::~Smartport()
{
    deleteSensors();
}

const uint8_t Smartport::sensorIdMatrix[29] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0xD, 0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B, 0x0};

void Smartport::begin()
{
#if defined(CONFIG_LUA) && RX_PROTOCOL == RX_SMARTPORT
    Config config = readConfig();
#else
    Config config;
#endif
    delay(100);
    setConfig(config);
}

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
}

void Smartport::sendVoid()
{
    const uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    serial_.write(buf, 8);
}

void Smartport::addSensor(Sensor *newSensorP)
{
    static Sensor *prevSensorP;
    if (sensorP == NULL)
    {
        prevSensorP = newSensorP;
        newSensorP->nextP = newSensorP;
    }
    sensorP = newSensorP;
    sensorP->nextP = prevSensorP->nextP;
    prevSensorP->nextP = newSensorP;
    prevSensorP = newSensorP;
}

bool Smartport::addPacket(uint16_t dataId, uint32_t value)
{
    return addPacket(0x10, dataId, value);
}

bool Smartport::addPacket(uint8_t frameId, uint16_t dataId, uint32_t value)
{
    if (packetP == NULL)
    {
        packetP = new Packet;
        packetP->frameId = frameId;
        packetP->dataId = dataId;
        packetP->value = value;
        return true;
    }
    return false;
}

void Smartport::deleteSensors()
{
    if (sensorP != NULL)
    {
        Sensor *firstSensorP;
        firstSensorP = sensorP;
        Sensor *nextSensorP;
        do
        {
            nextSensorP = sensorP->nextP;
            delete sensorP;
            sensorP = nextSensorP;
        } while (sensorP != firstSensorP);
        sensorP = NULL;
    }
    if (packetP != NULL)
    {
        delete packetP;
        packetP = NULL;
    }
}

uint8_t Smartport::read(uint8_t &sensorId, uint8_t &frameId, uint16_t &dataId, uint32_t &value)
{
    if (serial_.available())
    {
        uint8_t data[9];
        boolean header = false;
        uint8_t cont = 0;
        uint16_t tsRead = millis();
        while ((uint16_t)millis() - tsRead < SMARTPORT_TIMEOUT)
        {
            if (serial_.available())
            {
                tsRead = millis();
                if (serial_.peek() == 0x7E)
                {
                    header = true;
                    cont = 0;
                    serial_.read();
                }
                else
                {
                    data[cont] = serial_.read();
                    if (header)
                    {
                        if (data[cont] == 0x7D)
                            data[cont] = serial_.read() ^ 0x20;
                        cont++;
                    }
                }
                if (cont == 9)
                {
                    uint16_t crc = 0;
                    for (uint8_t i = 1; i < 8; i++)
                    {
                        crc += data[i];
                        crc += crc >> 8;
                        crc &= 0x00FF;
                    }
                    crc = 0xFF - (uint8_t)crc;
                    if (crc == data[8] && data[1] != 0x00 && data[1] != 0x10)
                    {
                        sensorId = data[0];
                        frameId = data[1];
                        dataId = (uint16_t)data[3] << 8 | data[2];
                        value = (uint32_t)data[7] << 24 | (uint32_t)data[6] << 16 |
                                (uint16_t)data[5] << 8 | data[4];
                        return RECEIVED_PACKET;
                    }
                    cont = 0;
                }
            }
        }
        if (cont == 1)
        {
            sensorId = data[0];
            return RECEIVED_POLL;
        }
    }
    return RECEIVED_NONE;
}

uint8_t Smartport::update()
{
    uint8_t frameId;
    uint16_t dataId;
    uint32_t value;
#ifdef SIM_RX
    if (true)
    {
        static uint16_t ts = 0;
        uint8_t sensorId = sensorId_;
        uint8_t packetType = RECEIVED_NONE;
        if (millis() - ts > 12)
        {
            packetType = RECEIVED_POLL;
            ts = millis();
        }
#else
    if (serial_.available())
    {
        uint8_t sensorId;
        uint8_t packetType = read(sensorId, frameId, dataId, value);
#endif
        if (packetType == RECEIVED_POLL && sensorId == sensorId_)
        {
            if (packetP != NULL && maintenanceMode_) // if maintenance send packet
            {
                sendData(packetP->frameId, packetP->dataId, packetP->value);
#ifdef DEBUG
                DEBUG_SERIAL.print(">F:");
                DEBUG_SERIAL.print(packetP->frameId, HEX);
                DEBUG_SERIAL.print(" D:");
                DEBUG_SERIAL.print(packetP->dataId, HEX);
                DEBUG_SERIAL.print(" V:");
                DEBUG_SERIAL.println(packetP->value, HEX);
#endif
                dataId = packetP->dataId;
                value = packetP->value;
                delete packetP;
                packetP = NULL;
                return SENT_PACKET;
            }
            if (sensorP != NULL && !maintenanceMode_) // else send telemetry
            {
                static Sensor *spSensorP = sensorP; // loop sensors until correct timestamp or 1 sensors cycle
                Sensor *initialSensorP = spSensorP;
                while (((uint16_t)((uint16_t)millis() - spSensorP->timestamp()) <= (uint16_t)spSensorP->refresh() * 100) && spSensorP->nextP != initialSensorP)
                {
                    spSensorP = spSensorP->nextP;
                }
                if ((uint16_t)((uint16_t)millis() - spSensorP->timestamp()) >= (uint16_t)spSensorP->refresh() * 100)
                {
                    sendData(spSensorP->frameId(), spSensorP->dataId(), spSensorP->valueFormatted());
#ifdef DEBUG
                    DEBUG_SERIAL.print("D:");
                    DEBUG_SERIAL.print(spSensorP->dataId(), HEX);
                    DEBUG_SERIAL.print(" V:");
                    DEBUG_SERIAL.print(spSensorP->valueFormatted());
                    DEBUG_SERIAL.print(" T:");
                    DEBUG_SERIAL.println(spSensorP->timestamp());
#endif
                    spSensorP->setTimestamp(millis());
                    dataId = spSensorP->dataId();
                    frameId = 0;
                    value = 0;
                    spSensorP = spSensorP->nextP;
                    return SENT_TELEMETRY;
                }
                else
                {
                    dataId = 0;
                    value = 0;
                    sendVoid();
                    return SENT_VOID;
                }
            }
        }
        else if (packetType == RECEIVED_PACKET)
        {
            processPacket(frameId, dataId, value);
#ifdef DEBUG
            DEBUG_SERIAL.print("<F:");
            DEBUG_SERIAL.print(frameId, HEX);
            DEBUG_SERIAL.print(" D:");
            DEBUG_SERIAL.print(dataId, HEX);
            DEBUG_SERIAL.print(" V:");
            DEBUG_SERIAL.println(value, HEX);
#endif
            return RECEIVED_PACKET;
        }
    }
    // update sensor
    if (sensorP != NULL)
    {
        sensorP->update();
        sensorP = sensorP->nextP;
    }
    return SENT_NONE;
}

bool Smartport::isSendPacketReady()
{
    if (packetP != NULL)
        return false;
    return true;
}

void Smartport::setConfig(Config &config)
{
    deleteSensors();

    setSensorId(idToCrc(config.sensorId));
    if (config.protocol == PROTOCOL_PWM)
    {
        Sensor *sensorP;
        EscPWM *esc;
        esc = new EscPWM(config.average.rpm);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        Sensor *sensorP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        Sensor *sensorP;
        EscHW4 *esc;
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), config.protocol - PROTOCOL_HW_V4_LV);
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->currentP(), esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->tempFetP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, esc->tempBecP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_CASTLE)
    {
        Sensor *sensorP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->currentP(), esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(SBEC_POWER_FIRST_ID, esc->becCurrentP(), esc->becVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID + 1, NULL, esc->rippleVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->temperatureP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        Sensor *sensorP;
        EscKontronik *esc;
        ESC_SERIAL.begin(115200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, esc->currentP(), esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new SensorDouble(SBEC_POWER_FIRST_ID, esc->becCurrentP(), esc->becVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, esc->tempFetP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, esc->tempBecP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.gps == true)
    {
        Sensor *sensorP;
        Bn220 *gps;
        GPS_SERIAL.begin(9600);
        GPS_SERIAL.setTimeout(BN220_TIMEOUT);
        gps = new Bn220(GPS_SERIAL);
        sensorP = new SensorLatLon(GPS_LONG_LATI_FIRST_ID, gps->lonP(), gps->latP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensor(GPS_ALT_FIRST_ID, gps->altP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensor(GPS_SPEED_FIRST_ID, gps->spdP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensor(GPS_COURS_FIRST_ID, gps->cogP(), 10, gps);
        addSensor(sensorP);
        sensorP = new SensorDateTime(GPS_TIME_DATE_FIRST_ID, gps->timeP(), gps->dateP(), 10, gps);
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
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt));
        sensorP = new Sensor(A3_FIRST_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
    }
    if (config.voltage2 == true)
    {
        Sensor *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt));
        sensorP = new Sensor(A4_FIRST_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
    }
    if (config.current == true)
    {
        Sensor *sensorP;
        Voltage *current;
        current = new Voltage(PIN_CURRENT, ALPHA(config.average.curr));
        sensorP = new Sensor(CURR_FIRST_ID, current->valueP(), config.refresh.curr, current);
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
    for (uint8_t i = 0; i < 2; i++)
    {
        if (config.deviceI2C[i].type == I2C_BMP280)
        {
            Sensor *sensorP;
            Bmp280 *bmp;
            bmp = new Bmp280(config.deviceI2C[i].address, ALPHA(config.average.temp), 10);
            bmp->begin();
            sensorP = new Sensor(T1_FIRST_ID + 2, bmp->temperatureP(), config.refresh.temp, bmp);
            addSensor(sensorP);
            sensorP = new Sensor(ALT_FIRST_ID + 2, bmp->altitudeP(), 10, bmp);
            addSensor(sensorP);
        }
    }
}

void Smartport::processPacket(uint8_t frameId, uint16_t dataId, uint32_t value)
{
    // maintenance mode on
    if (frameId == 0x21 && dataId == 0xFFFF && value == 0x80)
    {
#ifdef DEBUG
        DEBUG_SERIAL.println("M+");
#endif
        maintenanceMode_ = true;
        return;
    }

    // maintenance mode off
    if (frameId == 0x20 && dataId == 0xFFFF && value == 0x80)
    {
#ifdef DEBUG
        DEBUG_SERIAL.println("M-");
#endif
        maintenanceMode_ = false;
        return;
    }

    // send sensorId
    if (maintenanceMode_ && frameId == 0x30 && dataId == dataId_ && value == 0x01)
    {
#ifdef DEBUG
        DEBUG_SERIAL.print("S>");
        DEBUG_SERIAL.print(crcToId(sensorId_));
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
        DEBUG_SERIAL.print("S<");
        DEBUG_SERIAL.println(crcToId(sensorId_));
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
        while (!isSendPacketReady())
            update();
        addPacket(0x32, DATA_ID, value);
        // packet 2, 3 & 4
        for (uint8_t i = 0; i < 3; i++)
        {
            value = 0xF2 + i;
            memcpy(&value + 1, (uint8_t *)&config + 3 * i, 3);
            while (!isSendPacketReady())
                update();
            addPacket(0x32, DATA_ID, value);
        }
        return;
    }

    // receive config
    if (maintenanceMode_ && frameId == 0x31 && dataId == DATA_ID && (uint8_t)(value) == 0xF1)
    {

        Config config;
        for (uint8_t i = 0; i < 3; i++)
        {
#ifdef DEBUG
            DEBUG_SERIAL.print(i + 1);
            DEBUG_SERIAL.print(":");
            DEBUG_SERIAL.println(value);
#endif
            memcpy(&config + 3 * i, (uint8_t *)&value + 1, 3);
            while (frameId != 0x31 || dataId != DATA_ID || (uint8_t)(value) != 0xF1 + i)
                read(sensorId_, frameId, dataId, value);
        }
        while (!isSendPacketReady())
            update();
        addPacket(0x32, DATA_ID, 0xFF);
        while (!isSendPacketReady())
            update();
        PwmOut pwmOut;
        if (config.pwmOut)
            pwmOut.enable();
        else
            pwmOut.disable();
        writeConfig(config);
        setConfig(config);
    }
}