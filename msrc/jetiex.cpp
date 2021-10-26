#include "jetiex.h"

JetiEx::JetiEx(AbstractSerial &serial) : serial_(serial)
{
}

JetiEx::~JetiEx()
{
}

void JetiEx::begin()
{
    serial_.begin(baudRate, SERIAL_8N1 | SERIAL_HALF_DUP);
    serial_.setTimeout(2);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

uint8_t JetiEx::addSensor(SensorJetiEx *newSensorJetiExP)
{
    static uint8_t number = 1;
    sensorJetiExP[number] = newSensorJetiExP;
    if (number < 16)
        number++;
    return number - 1;
}

bool JetiEx::addSensorValueToBuffer(uint8_t *buffer, uint8_t &posBuffer, uint8_t &sensorNumber)
{
    if (sensorJetiExP[sensorNumber])
    {
        uint8_t format = sensorJetiExP[sensorNumber]->decimals() << 5;
        if (sensorJetiExP[sensorNumber]->valueP() < 0)
        {
            *sensorJetiExP[sensorNumber]->valueP() *= -1;
            format |= 1 << 7;
        }
        if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT6)
        {
            uint8_t value = *sensorJetiExP[sensorNumber]->valueP() * pow(10, sensorJetiExP[sensorNumber]->decimals());
            if (value > 0x1F)
                value = 0x1F;
            value |= format;
            if (posBuffer > 25)
                return false;
            else
            {
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                posBuffer += 2;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT14)
        {
            uint16_t value = *sensorJetiExP[sensorNumber]->valueP() * pow(10, sensorJetiExP[sensorNumber]->decimals());
            if (value > 0x1FFF)
                value = 0x1FFF;
            value |= format << 8;
            if (posBuffer > 24)
                return false;
            else
            {
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                buffer[posBuffer + 2] = value >> 8;
                posBuffer += 3;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT22)
        {
            uint32_t value = *sensorJetiExP[sensorNumber]->valueP() * pow(10, sensorJetiExP[sensorNumber]->decimals());
            if (value > 0x1FFFFF)
                value = 0x1FFFFF;
            value |= (uint32_t)format << 16;
            if (posBuffer > 23)
                return false;
            else
            {
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                buffer[posBuffer + 2] = value >> 8;
                buffer[posBuffer + 3] = value >> 16;
                posBuffer += 4;
            }
        }
    }
    else
        return false;
    sensorNumber++;
    if (sensorJetiExP[sensorNumber] == NULL)
        sensorNumber = 1;
    return true;
}

bool JetiEx::addSensorTextToBuffer(uint8_t *buffer, uint8_t &posBuffer, uint8_t &sensorNumber)
{
    if (sensorJetiExP[sensorNumber])
    {
        uint8_t lenText = strlen(sensorJetiExP[sensorNumber]->textP());
        uint8_t lenUnit = strlen(sensorJetiExP[sensorNumber]->unitP());

        if (posBuffer + lenText + lenUnit + 2 < 28)
        {
            buffer[posBuffer] = sensorNumber;
            buffer[posBuffer + 1] = lenText << 3 | lenUnit;
            posBuffer += 2;
            strcpy((char *)buffer + posBuffer, sensorJetiExP[sensorNumber]->textP());
            posBuffer += lenText;
            strcpy((char *)buffer + posBuffer, sensorJetiExP[sensorNumber]->unitP());
            posBuffer += lenUnit;
            sensorNumber++;
            if (sensorJetiExP[sensorNumber] == NULL)
                sensorNumber = 1;
            return true;
        }
    }
    return false;
}

uint8_t JetiEx::createExBuffer(uint8_t *buffer, bool sendValue)
{
    static uint8_t sensorNumberTelemetry = 1;
    static uint8_t sensorNumberText = 1;
    uint8_t posBuffer = 7;
    if (sensorJetiExP[1] == NULL)
        return 0;
    if (sendValue)
    {
        uint8_t firstSensor = sensorNumberTelemetry;
        while (addSensorValueToBuffer(buffer, posBuffer, sensorNumberTelemetry) && sensorNumberTelemetry != firstSensor)
            ;
        buffer[1] = 0x40;
    }
    else
    {
        uint8_t firstSensor = sensorNumberText;
        while (addSensorTextToBuffer(buffer, posBuffer, sensorNumberText) && sensorNumberText != firstSensor)
            ;
    }
    buffer[0] = 0x0F;
    buffer[1] |= posBuffer - 1;
    buffer[2] = JETIEX_MFG_ID_LOW;
    buffer[3] = JETIEX_MFG_ID_HIGH;
    buffer[4] = JETIEX_DEV_ID_LOW;
    buffer[5] = JETIEX_DEV_ID_HIGH;
    buffer[6] = 0x00;
    buffer[posBuffer] = crc8(buffer + 1, posBuffer - 1);
    return posBuffer + 1;
}

void JetiEx::sendPacket(uint8_t packetId)
{
    digitalWrite(LED_BUILTIN, HIGH);
    static uint8_t packetCount = 0;
    uint8_t buffer[36] = {0};
    uint8_t lengthExBuffer = createExBuffer(buffer + 6, packetCount % 16);
    buffer[0] = 0x3B;
    buffer[1] = 0x01;
    buffer[2] = lengthExBuffer + 8;
    buffer[3] = packetId;
    buffer[4] = 0x3A;
    buffer[5] = lengthExBuffer;
    uint16_t crc = crc16(buffer, lengthExBuffer + 6);
    buffer[lengthExBuffer + 6] = crc;
    buffer[lengthExBuffer + 7] = crc >> 8;
    serial_.writeBytes(buffer, lengthExBuffer + 8);
#ifdef DEBUG
    if (packetCount % 16)
        DEBUG_PRINT("V"); // values
    else
        DEBUG_PRINT("T"); // text
    DEBUG_PRINT(">");
    for (uint8_t i = 0; i < lengthExBuffer + 8; i++)
    {
        DEBUG_PRINT_HEX(buffer[i]);
        DEBUG_PRINT(" ");
        delay(1);
    }
    DEBUG_PRINTLN();
#endif
    packetCount++;
    digitalWrite(LED_BUILTIN, LOW);
}

void JetiEx::update()
{
    uint8_t status = JETIEX_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    static uint8_t packetId = 0;
    if ((uint16_t)millis() - ts > 100)
    {
        if (!mute)
        {
            status = JETIEX_SEND;
            packetId++;
        }
        mute = !mute;
        ts = millis();
    }
#else
    uint8_t packetId;
    uint8_t length = serial_.availableTimeout();
    uint8_t buff[length];
    static uint8_t badCrcCount = 0;
    serial_.readBytes(buff, length);
    if (length)
    {
#ifdef DEBUG
        DEBUG_PRINT("<");
        for (uint8_t i = 0; i < JETIEX_PACKET_LENGHT; i++)
        {
            DEBUG_PRINT_HEX(buff[i]);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINTLN();
#endif
        if (crc16(buff, length) == 0)
        {
            badCrcCount = 0;
            if (length == JETIEX_PACKET_LENGHT && buff[0] == 0x3D && buff[1] == 0x01 && buff[4] == 0x3A)
            {
                if (!mute)
                {
                    status = JETIEX_SEND;
                    packetId = buff[3];
                }
                mute = !mute;
            }
        }
        else
        {
            badCrcCount++;
            if (badCrcCount > 5)
            {

                if (baudRate == 125000L)
                    baudRate = 250000L;
                else
                    baudRate = 125000L;
                serial_.begin(baudRate, SERIAL_8N1 | SERIAL_HALF_DUP);
                badCrcCount = 0;
#ifdef DEBUG
                DEBUG_PRINT("BR:");
                DEBUG_PRINT(baudRate);
                DEBUG_PRINTLN();
#endif
            }
        }
    }
#endif
    if (status == JETIEX_SEND)
    {
        sendPacket(packetId);
    }

    // update sensor
    static uint8_t cont = 0;
    if (sensorJetiExP[cont])
    {
        sensorJetiExP[cont]->update();
    }
    cont++;
    if (cont == 16 || sensorJetiExP == NULL)
        cont = 0;
}

void JetiEx::setConfig(Config &config)
{
    if (config.protocol == PROTOCOL_PWM)
    {
        SensorJetiEx *sensorJetiExP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        SensorJetiEx *sensorJetiExP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        SensorJetiEx *sensorJetiExP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), config.protocol - PROTOCOL_HW_V4_LV);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->currentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, esc->tempFetP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp FET");
        sensorJetiExP->setUnit("°C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, esc->tempBecP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp BEC");
        sensorJetiExP->setUnit("°C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
    }
    if (config.protocol == PROTOCOL_CASTLE)
    {
        SensorJetiEx *sensorJetiExP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->currentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->rippleVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Ripple Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->becCurrentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("BEC Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->becVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("BEC Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, esc->temperatureP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temperature");
        sensorJetiExP->setUnit("°C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        SensorJetiEx *sensorJetiExP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->currentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->becCurrentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("BEC Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->becVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("BEC Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, esc->tempFetP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp FET");
        sensorJetiExP->setUnit("°C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, esc->tempBecP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp BEC");
        sensorJetiExP->setUnit("°C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
    }
    if (config.gps == true)
    {
        SensorJetiEx *sensorJetiExP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, gps->spdP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Speed");
        sensorJetiExP->setUnit("kts");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, gps->altP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Altitude");
        sensorJetiExP->setUnit("m");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_TIMEDATE, 0, gps->timeP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Time");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_COORDINATES, 0, gps->latP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Latitude");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_COORDINATES, 0, gps->lonP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Longitude");
    }
    if (config.airspeed == true)
    {
        SensorJetiEx *sensorJetiExP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, pressure->valueP(), pressure);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Vario");
        sensorJetiExP->setUnit("m/s");
    }
    if (config.voltage1 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, voltage->valueP(), voltage);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage 1");
        sensorJetiExP->setUnit("V");
    }
    if (config.voltage2 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, voltage->valueP(), voltage);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage 2");
        sensorJetiExP->setUnit("V");
    }
    if (config.current == true)
    {
        SensorJetiEx *sensorJetiExP;
        Voltage *current;
        current = new Voltage(PIN_CURRENT, ALPHA(config.average.curr));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, current->valueP(), current);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
    }
    if (config.ntc1 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, ntc->valueP(), ntc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp 1");
        sensorJetiExP->setUnit("°C");
    }
    if (config.ntc2 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, 0, ntc->valueP(), ntc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp 2");
        sensorJetiExP->setUnit("°C");
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        SensorJetiEx *sensorJetiExP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), 10);
        bmp->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->altitudeP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Altitude");
        sensorJetiExP->setUnit("m");
    }
}

uint8_t JetiEx::crc8(uint8_t *crc, uint8_t crc_length)
{
    uint8_t crc_up = 0;
    uint8_t c;
    for (c = 0; c < crc_length; c++)
    {
        crc_up = update_crc8(crc[c], crc_up);
    }
    return crc_up;
}

uint8_t JetiEx::update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u;
    uint8_t i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
    {
        crc_u = (crc_u & 0x80) ? 0x07 ^ (crc_u << 1) : (crc_u << 1);
    }
    return crc_u;
}

uint16_t JetiEx::crc16(uint8_t *p, uint16_t len)
{
    uint16_t crc16_data = 0;
    while (len--)
    {
        crc16_data = update_crc16(crc16_data, p[0]);
        p++;
    }
    return (crc16_data);
}

uint16_t JetiEx::update_crc16(uint16_t crc, uint8_t data)
{
    uint16_t ret_val;
    data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
    data ^= data << 4;
    ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
    return ret_val;
}