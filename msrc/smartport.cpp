#include "smartport.h"

AbstractDevice::AbstractDevice() {}

float AbstractDevice::calcAverage(float alpha, float value, float newValue)
{
    return value + alpha * (newValue - value);
}

Sensor::Sensor(uint16_t dataId, uint8_t indexM, uint8_t indexL, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), indexL_(indexL), indexM_(indexM), refresh_(refresh), device_(device) {}
Sensor::Sensor(uint16_t dataId, uint8_t indexL, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), indexL_(indexL), refresh_(refresh), device_(device) {}
Sensor::Sensor(uint16_t dataId, uint8_t refresh, AbstractDevice *device) : dataId_(dataId), refresh_(refresh), device_(device) {}

float Sensor::read(uint8_t index)
{
    return device_->read(index);
}

uint8_t Sensor::indexL()
{
    return indexL_;
}

uint8_t Sensor::indexM()
{
    return indexM_;
}

uint16_t Sensor::timestamp()
{
    return timestamp_;
}

void Sensor::setTimestamp(uint16_t timestamp)
{
    timestamp_ = timestamp;
}

uint16_t Sensor::dataId()
{
    return dataId_;
}

uint16_t Sensor::frameId()
{
    return frameId_;
}

uint8_t Sensor::refresh()
{
    return refresh_;
}

float Sensor::valueM()
{
    return valueM_;
}

void Sensor::setValueM(float value)
{
    valueM_ = value;
}

float Sensor::valueL()
{
    return valueL_;
}

void Sensor::setValueL(float value)
{
    valueL_ = value;
}

Smartport::Smartport(Stream &serial) : serial_(serial)
{
    pinMode(LED_SMARTPORT, OUTPUT);
}

Smartport::~Smartport()
{
    deleteSensors();
}

uint8_t Smartport::sensorId()
{
    return sensorId_;
}

void Smartport::setSensorId(uint8_t sensorId)
{
    sensorId_ = sensorId;
}

void Smartport::setDataId(uint16_t dataId)
{
    dataId_ = dataId;
}

uint8_t Smartport::maintenanceMode()
{
    return maintenanceMode_;
}

void Smartport::setMaintenanceMode(uint8_t maintenanceMode)
{
    maintenanceMode_ = maintenanceMode;
}

uint8_t Smartport::available()
{
    return serial_.available();
}

uint8_t Smartport::idToCrc(uint8_t sensorId)
{
    const uint8_t sensorIdMatrix[28] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0xD, 0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B};
    if (sensorId < 1 || sensorId > 28)
    {
        return 0;
    }
    return sensorIdMatrix[sensorId - 1];
}

uint8_t Smartport::crcToId(uint8_t sensorIdCrc)
{
    const uint8_t sensorIdMatrix[29] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0xD, 0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B, 0x0};
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
    uint16_t crc = *crcp;

    if (crcp != NULL)
    {
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
    digitalWrite(LED_SMARTPORT, HIGH);
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
    digitalWrite(LED_SMARTPORT, LOW);
}

void Smartport::sendVoid()
{
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    serial_.write(buf, 8);
}

uint32_t Smartport::formatData(uint16_t dataId, float valueM, float valueL)
{

    if ((dataId >= GPS_SPEED_FIRST_ID && dataId <= GPS_SPEED_LAST_ID) ||
        (dataId >= RBOX_BATT1_FIRST_ID && dataId <= RBOX_BATT2_FIRST_ID))
        return round(valueL * 1000);

    if ((dataId >= ALT_FIRST_ID && dataId <= VARIO_LAST_ID) ||
        (dataId >= VFAS_FIRST_ID && dataId <= VFAS_LAST_ID) ||
        (dataId >= ACCX_FIRST_ID && dataId <= GPS_ALT_LAST_ID) ||
        (dataId >= GPS_COURS_FIRST_ID && dataId <= GPS_COURS_LAST_ID) ||
        (dataId >= A3_FIRST_ID && dataId <= A4_LAST_ID))
        return round(valueL * 100);

    if ((dataId >= CURR_FIRST_ID && dataId <= CURR_LAST_ID) ||
        (dataId >= AIR_SPEED_FIRST_ID && dataId <= AIR_SPEED_LAST_ID) ||
        dataId == A1_ID || dataId == A2_ID || dataId == RXBT_ID)
        return round(valueL * 10);

    if (dataId >= ESC_POWER_FIRST_ID && dataId <= ESC_POWER_LAST_ID)
        return (uint32_t)round(valueM * 100) << 16 | (uint16_t)round(valueL * 100);

    if (dataId >= ESC_RPM_CONS_FIRST_ID && dataId <= ESC_RPM_CONS_LAST_ID)
    {
        return (uint32_t)round(valueM) << 16 | (uint16_t)round((valueL) / 100);
    }

    if (dataId >= SBEC_POWER_FIRST_ID && dataId <= SBEC_POWER_LAST_ID)
        return (uint32_t)round(valueM * 1000) << 16 | (uint16_t)round((valueL)*1000);

    if (dataId >= CELLS_FIRST_ID && dataId <= CELLS_LAST_ID)
        return (uint16_t)round(valueM * 500) << 8 | (uint16_t)valueL;

    return round(valueL);
}

uint32_t Smartport::formatData(uint16_t dataId, float valueL)
{
    return formatData(dataId, 0, valueL);
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
        Sensor *firstSensorP = sensorP;
        Sensor *nextSensorP;
        do
        {
            nextSensorP = sensorP->nextP;
            delete sensorP;
            sensorP = nextSensorP;
        } while (sensorP != firstSensorP);
        sensorP = NULL;
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
                else {
                    data[cont] = serial_.read();
                    if (header) {
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
                    if (crc == data[8])
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

uint8_t Smartport::update(uint8_t &frameId, uint16_t &dataId, uint32_t &value)
{
#ifdef SIM_POLL
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
    if (available())
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
                Serial.print("Sent frameId: ");
                Serial.print(packetP->frameId, HEX);
                Serial.print(" dataId: ");
                Serial.print(packetP->dataId, HEX);
                Serial.print(" value: ");
                Serial.println(packetP->value);
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
                while (((uint16_t)millis() - spSensorP->timestamp() <= (uint16_t)spSensorP->refresh() * 100) && spSensorP->nextP != initialSensorP)
                {
                    spSensorP = spSensorP->nextP;
                }
                if ((uint16_t)millis() - spSensorP->timestamp() >= (uint16_t)spSensorP->refresh() * 100)
                {
                    sendData(spSensorP->frameId(), spSensorP->dataId(), formatData(spSensorP->dataId(), spSensorP->valueM(), spSensorP->valueL()));
#ifdef DEBUG2
                    Serial.print("id: ");
                    Serial.print(spSensorP->dataId(), HEX);
                    Serial.print(" iL: ");
                    Serial.print(spSensorP->indexL());
                    Serial.print(" vL: ");
                    Serial.print(spSensorP->valueL());
                    Serial.print(" f: ");
                    Serial.print(formatData(spSensorP->dataId(), spSensorP->valueM(), spSensorP->valueL()));
                    Serial.print(" ts: ");
                    Serial.println(spSensorP->timestamp());
                    if (spSensorP->indexM() != 255)
                    {
                        Serial.print("id: ");
                        Serial.print(spSensorP->dataId(), HEX);
                        Serial.print(" iM: ");
                        Serial.print(spSensorP->indexM());
                        Serial.print(" vM: ");
                        Serial.print(spSensorP->valueM());
                        Serial.print(" ts: ");
                        Serial.println(spSensorP->timestamp());
                    }
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
        else if (packetType == RECEIVED_PACKET && frameId != 0x10)
        {
            // maintenance mode on
            if (frameId == 0x21 && dataId == 0xFFFF && value == 0x80)
            {
#ifdef DEBUG
                Serial.println("Maintenance mode ON");
#endif
                maintenanceMode_ = true;
                return MAINTENANCE_ON;
            }
            // maintenance mode off
            if (frameId == 0x20 && dataId == 0xFFFF && value == 0x80)
            {
#ifdef DEBUG
                Serial.println("Maintenance mode OFF");
#endif
                maintenanceMode_ = false;
                return MAINTENANCE_OFF;
            }
            // send sensorId
            if (maintenanceMode_ && frameId == 0x30 && dataId == dataId_ && value == 1)
            {
#ifdef DEBUG
                Serial.print("Sent Sensor Id: ");
                Serial.print(crcToId(sensorId_));
                Serial.print(" ");
                Serial.println(sensorId_, HEX);
#endif
                addPacket(0x32, dataId_, 256 * (crcToId(sensorId_) - 1) + 1);
                return SENT_SENSOR_ID;
            }
            // change sensorId
            if (maintenanceMode_ && frameId == 0x31 && dataId == dataId_)
            {
                setSensorId(idToCrc(((value - 1) / 256) + 1));
#ifdef DEBUG
                Serial.print("Changed Sensor Id: ");
                Serial.println(crcToId(sensorId_));
#endif
                return CHANGED_SENSOR_ID;
            }
#ifdef DEBUG
            Serial.print("Received frameId: ");
            Serial.print(frameId, HEX);
            Serial.print(" dataId: ");
            Serial.print(dataId, HEX);
            Serial.print(" value: ");
            Serial.println(value);
#endif
            return RECEIVED_PACKET;
        }
    }
    // update sensor
    if (sensorP != NULL)
    {
        if (sensorP->indexL() != 255)
            sensorP->setValueL(sensorP->read(sensorP->indexL()));
        if (sensorP->indexM() != 255)
            sensorP->setValueM(sensorP->read(sensorP->indexM()));
        sensorP = sensorP->nextP;
    }
    return SENT_NONE;
}

uint8_t Smartport::update()
{
    uint8_t frameId;
    uint16_t dataId;
    uint32_t value;
    return update(frameId, dataId, value);
}

bool Smartport::sendPacketReady()
{
    if (packetP != NULL)
        return false;
    return true;
}

/*void Smartport::begin(uint32_t baudRate, uint8_t sensorId, uint8_t sensorIdTx)
{
    sensorId = getSensorIdMatrix(sensorId);
    sensorIdTx = getSensorIdMatrix(sensorIdTx);
    //serial_.begin(baudRate);
}*/
