#include "jetiex.h"

JetiEx::JetiEx(AbstractSerial &serial) : serial_(serial)
{
}

JetiEx::~JetiEx()
{
}

void JetiEx::begin()
{
    serial_.begin(baudRate, SERIAL__8N1 | SERIAL__HALF_DUP);
    serial_.setTimeout(JETIEX_TIMEOUT);
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
        uint8_t format = sensorJetiExP[sensorNumber]->format() << 5;
        if (*sensorJetiExP[sensorNumber]->valueP() < 0)
        {
            if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_COORDINATES)
                format |= 1 << 6;
            else
                format |= 1 << 7;
        }
        if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT6)
        {
            if (posBuffer > 25)  // 29 bytes max:  25+2=pos27=byte28 +1crc=byte29
                return false;
            else
            {
                uint8_t value = abs(*sensorJetiExP[sensorNumber]->valueP()) * pow(10, sensorJetiExP[sensorNumber]->format());
                if (value > 0x1F)
                    value = 0x1F;
                value |= format;
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                posBuffer += 2;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT14)
        {
            if (posBuffer > 24)
                return false;
            else
            {
                uint16_t value = abs(*sensorJetiExP[sensorNumber]->valueP()) * pow(10, sensorJetiExP[sensorNumber]->format());
                if (value > 0x1FFF)
                    value = 0x1FFF;
                value |= (uint16_t)format << 8;
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                buffer[posBuffer + 2] = value >> 8;
                posBuffer += 3;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT22)
        {
            if (posBuffer > 23)
                return false;
            else
            {
                uint32_t value = abs(*sensorJetiExP[sensorNumber]->valueP()) * pow(10, sensorJetiExP[sensorNumber]->format());
                if (value > 0x1FFFFF)
                    value = 0x1FFFFF;
                value |= (uint32_t)format << 16;
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                buffer[posBuffer + 2] = value >> 8;
                buffer[posBuffer + 3] = value >> 16;
                posBuffer += 4;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_INT30)
        {
            if (posBuffer > 22)
                return false;
            else
            {
                uint32_t value = abs(*sensorJetiExP[sensorNumber]->valueP()) * pow(10, sensorJetiExP[sensorNumber]->format());
                if (value > 0x1FFFFFFF)
                    value = 0x1FFFFFFF;
                value |= (uint32_t)format << 24;
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = value;
                buffer[posBuffer + 2] = value >> 8;
                buffer[posBuffer + 3] = value >> 16;
                buffer[posBuffer + 4] = value >> 24;
                posBuffer += 5;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_TIMEDATE)
        {
            if (posBuffer > 23)
                return false;
            else
            {
                // rawvalue: yymmdd/hhmmss
                // byte 1: day/second
                // byte 2: month/minute
                // byte 3(bits 1-5): year/hour
                // byte 3(bit 6): 0=time 1=date
                uint32_t value = *sensorJetiExP[sensorNumber]->valueP();
                uint8_t hourYearFormat = format;
                hourYearFormat |= value / 10000;                             // hour, year
                uint8_t minuteMonth = (value / 100 - (value / 10000) * 100); // minute, month
                uint8_t secondDay = value - (value / 100) * 100;             // second, day
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                buffer[posBuffer + 1] = secondDay;
                buffer[posBuffer + 2] = minuteMonth;
                buffer[posBuffer + 3] = hourYearFormat;
                posBuffer += 4;
            }
        }
        else if (sensorJetiExP[sensorNumber]->type() == JETIEX_TYPE_COORDINATES)
        {
            if (posBuffer > 22)
                return false;
            else
            {
                // rawvalue: minutes
                // byte 1-2: degrees (decimals)
                // byte 3: degrees (integer)
                // byte 4(bit 6): 0=lat 1=lon
                // byte 4(bit 7): 0=+(N,E), 1=-(S,W)
                float value = abs(*sensorJetiExP[sensorNumber]->valueP());
                buffer[posBuffer] = sensorNumber << 4 | sensorJetiExP[sensorNumber]->type();
                uint8_t degrees = value / 60;
                uint16_t degreesDecimals = (value / 60 - degrees) * 10000;
                buffer[posBuffer + 1] = degreesDecimals;      // degrees (dec, l)
                buffer[posBuffer + 2] = degreesDecimals >> 8; // degrees (dec, h)
                buffer[posBuffer + 3] = degrees;              // degrees (int)
                buffer[posBuffer + 4] = format;               // format
                posBuffer += 5;
            }
        }
    }
    else
        return false;
    sensorNumber++;
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
        while (addSensorValueToBuffer(buffer, posBuffer, sensorNumberTelemetry) && sensorJetiExP[sensorNumberTelemetry] != NULL)
            ;
        if (sensorJetiExP[sensorNumberTelemetry] == NULL)
            sensorNumberTelemetry = 1;
        buffer[1] = 0x40;
    }
    else
    {
        /*while*/ (addSensorTextToBuffer(buffer, posBuffer, sensorNumberText) && sensorJetiExP[sensorNumberText] != NULL)
            ;
        if (sensorJetiExP[sensorNumberText] == NULL)
            sensorNumberText = 1;
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
        delayMicroseconds(100);
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
    if ((uint16_t)(millis() - ts) > 100)
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
    static uint16_t ts = 0;
    if (length)
    {
        uint8_t buff[length];
        serial_.readBytes(buff, length);
#ifdef DEBUG_PACKET2
        DEBUG_PRINT("<");
        for (uint8_t i = 0; i < length; i++)
        {
            DEBUG_PRINT_HEX(buff[i]);
            DEBUG_PRINT(" ");
            delayMicroseconds(100);
        }
        DEBUG_PRINTLN();
#endif
        uint8_t packet[JETIEX_PACKET_LENGHT];
        if (buff[0] == 0x3E && buff[1] == 0x3 && length - buff[2] == JETIEX_PACKET_LENGHT)
        {
            memcpy(packet, buff + buff[2], JETIEX_PACKET_LENGHT);
#ifdef DEBUG_PACKET
            DEBUG_PRINT("P:");
            for (uint8_t i = 0; i < JETIEX_PACKET_LENGHT; i++)
            {
                DEBUG_PRINT_HEX(packet[i]);
                DEBUG_PRINT(" ");
            }
            DEBUG_PRINTLN();
#endif
        }
        else if (length == JETIEX_PACKET_LENGHT)
        {
            memcpy(packet, buff, JETIEX_PACKET_LENGHT);
        }
        else
        {
            return;
        }
        if (crc16(packet, JETIEX_PACKET_LENGHT) == 0)
        {
            ts = millis();
            if (packet[0] == 0x3D && packet[1] == 0x01 && packet[4] == 0x3A)
            {
                if (!mute)
                {
                    status = JETIEX_SEND;
                    packetId = packet[3];
                }
                mute = !mute;
            }
        }
    }
#endif
    if (status == JETIEX_SEND)
    {
        if (serial_.timestamp() < 1500)
            sendPacket(packetId);
#ifdef DEBUG
        else
        {
            DEBUG_PRINT("KO");
            DEBUG_PRINTLN();
        }
#endif
    }
    if ((uint16_t)(millis() - ts) > 5000)
    {
        if (baudRate == 125000L)
            baudRate = 250000L;
        else
            baudRate = 125000L;
        serial_.begin(baudRate, SERIAL__8N1 | SERIAL__HALF_DUP);
        ts = millis();
#ifdef DEBUG
        DEBUG_PRINT("BR:");
        DEBUG_PRINT(baudRate);
        DEBUG_PRINTLN();
#endif
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
    if (ESC_PROTOCOL == PROTOCOL_PWM)
    {
        SensorJetiEx *sensorJetiExP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V3)
    {
        SensorJetiEx *sensorJetiExP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        SensorJetiEx *sensorJetiExP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->currentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->tempFetP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp FET");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->tempBecP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp BEC");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        SensorJetiEx *sensorJetiExP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
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
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->temperatureP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temperature");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        SensorJetiEx *sensorJetiExP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
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
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->tempFetP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp FET");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->tempBecP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp BEC");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        SensorJetiEx *sensorJetiExP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->currentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->tempP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temperature");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        SensorJetiEx *sensorJetiExP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("RPM");
        sensorJetiExP->setUnit("RPM");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, esc->currentP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->tempP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temperature");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Cell Voltage");
        sensorJetiExP->setUnit("V");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
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
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT22, 1, gps->altP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Altitude");
        sensorJetiExP->setUnit("m");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_TIMEDATE, JETIEX_FORMAT_TIME, gps->timeP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Time");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_TIMEDATE, JETIEX_FORMAT_DATE, gps->dateP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Date");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_COORDINATES, JETIEX_FORMAT_LAT, gps->latP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Latitude");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_COORDINATES, JETIEX_FORMAT_LON, gps->lonP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Longitude");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT6, JETIEX_FORMAT_0_DECIMAL, gps->satP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Sats");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, gps->hdopP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("HDOP");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, gps->varioP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Vario");
        sensorJetiExP->setUnit("m/s");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, gps->distP(), gps);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Dist Home");
        sensorJetiExP->setUnit("m");
    }
    if (config.airspeed == true)
    {
        SensorJetiEx *sensorJetiExP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, pressure->valueP(), pressure);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Air speed");
        sensorJetiExP->setUnit("m/s");
    }
    if (config.voltage1 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, voltage->valueP(), voltage);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage 1");
        sensorJetiExP->setUnit("V");
    }
    if (config.voltage2 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt), VOLTAGE2_MULTIPLIER);
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 2, voltage->valueP(), voltage);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Voltage 2");
        sensorJetiExP->setUnit("V");
    }
    if (config.current == true)
    {
        SensorJetiEx *sensorJetiExP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, current->valueP(), current);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Current");
        sensorJetiExP->setUnit("A");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, current->consumptionP(), current);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Consumption");
        sensorJetiExP->setUnit("mAh");
    }
    if (config.ntc1 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, ntc->valueP(), ntc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp 1");
        sensorJetiExP->setUnit("C");
    }
    if (config.ntc2 == true)
    {
        SensorJetiEx *sensorJetiExP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 0, ntc->valueP(), ntc);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temp 2");
        sensorJetiExP->setUnit("C");
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        SensorJetiEx *sensorJetiExP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->altitudeP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Altitude");
        sensorJetiExP->setUnit("m");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->temperatureP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temperature");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->varioP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Vario");
        sensorJetiExP->setUnit("m/s");
    }
    if (config.deviceI2C1Type == I2C_MS5611)
    {
        SensorJetiEx *sensorJetiExP;
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->altitudeP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Altitude");
        sensorJetiExP->setUnit("m");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->temperatureP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Temperature");
        sensorJetiExP->setUnit("C");
        sensorJetiExP = new SensorJetiEx(JETIEX_TYPE_INT14, 1, bmp->varioP(), bmp);
        sensorJetiExP->setSensorId(addSensor(sensorJetiExP));
        sensorJetiExP->setText("Vario");
        sensorJetiExP->setUnit("m/s");
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