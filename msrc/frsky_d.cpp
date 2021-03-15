#include "frsky_d.h"

Frsky::Frsky(Stream &serial) : serial_(serial)
{
}

Frsky::~Frsky()
{
    deleteSensors();
}

void Frsky::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Frsky::sendByte(uint8_t c, bool header)
{
    if ((c == 0x5D || c == 0x5E) && !header)
    {
        serial_.write(0x5D);
        Serial.print("X");
        c ^= 0x60;
    }
    serial_.write(c);
    Serial.print(c, HEX);
    Serial.print(" ");

}

void Frsky::sendData(uint8_t dataId, uint16_t value)
{
    digitalWrite(LED_BUILTIN, HIGH);
    uint8_t *u8p;
    // header
    sendByte(0x5E, true);
    // dataId
    sendByte(dataId, false);
    // value
    u8p = (uint8_t *)&value;
    sendByte(u8p[0], false);
    sendByte(u8p[1], false);
    // footer
    sendByte(0x5E, true);
    Serial.println();

    digitalWrite(LED_BUILTIN, LOW);
}

void Frsky::addSensor(Sensord *newSensorP)
{
    static Sensord *prevSensorP;
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

void Frsky::deleteSensors()
{
    if (sensorP != NULL)
    {
        Sensord *firstSensorP, *nextSensorP;
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
}

void Frsky::update()
{
    if (sensorP != NULL) // send telemetry
    {
        static Sensord *spSensorP = sensorP; // loop sensors until correct timestamp or 1 sensors cycle
        Sensord *initialSensorP = spSensorP;
        while (((uint16_t)((uint16_t)millis() - spSensorP->timestamp()) <= (uint16_t)spSensorP->refresh() * 100) && spSensorP->nextP != initialSensorP)
        {
            spSensorP = spSensorP->nextP;
        }
        if ((uint16_t)((uint16_t)millis() - spSensorP->timestamp()) >= (uint16_t)spSensorP->refresh() * 100)
        {
            sendData(spSensorP->dataId(), spSensorP->valueFormatted());
#ifdef DEBUG
            DEBUG_SERIAL.print("D:");
            DEBUG_SERIAL.print(spSensorP->dataId(), HEX);
            DEBUG_SERIAL.print(" V:");
            DEBUG_SERIAL.print(spSensorP->valueFormatted());
            spSensorP->valueFormatted(); // toggle type if date/time or lat/lon sensor
            DEBUG_SERIAL.print(" T:");
            DEBUG_SERIAL.println(spSensorP->timestamp());
#endif
            spSensorP->setTimestamp(millis());
            spSensorP = spSensorP->nextP;
        }
    }
    // update sensor
    if (sensorP != NULL)
    {
        sensorP->update();
        sensorP = sensorP->nextP;
    }
}

void Frsky::setConfig(Config &config)
{
    deleteSensors();
    if (config.protocol == PROTOCOL_PWM)
    {
        Sensord *sensorP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorP = new Sensord(RPM_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        Sensord *sensorP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorP = new Sensord(RPM_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        Sensord *sensorP;
        EscHW4 *esc;
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), config.protocol - PROTOCOL_HW_V4_LV);
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorP = new Sensord(RPM_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_BP_ID, esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_AP_ID, esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensord(CURRENT_ID, esc->currentP(), config.refresh.curr, esc);
        addSensor(sensorP);
        sensorP = new Sensord(TEMP1_ID, esc->tempFetP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensord(TEMP2_ID + 1, esc->tempBecP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VFAS_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_CASTLE)
    {
        Sensord *sensorP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorP = new Sensord(RPM_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_BP_ID, esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_AP_ID, esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensord(CURRENT_ID, esc->currentP(), config.refresh.curr, esc);
        addSensor(sensorP);
        //sensorP = new Sensord(SBEC_POWER_FIRST_ID, esc->becVoltageP(), esc->becCurrentP(), config.refresh.volt, esc);
        //addSensor(sensorP);
        //sensorP = new Sensord(ESC_POWER_FIRST_ID + 1, esc->rippleVoltageP(), NULL, config.refresh.volt, esc);
        //addSensor(sensorP);
        sensorP = new Sensord(TEMP1_ID, esc->temperatureP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VFAS_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        Sensord *sensorP;
        EscKontronik *esc;
        ESC_SERIAL.begin(115200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorP = new Sensord(RPM_ID, esc->rpmP(), config.refresh.rpm, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_BP_ID, esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_AP_ID, esc->voltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
        sensorP = new Sensord(CURRENT_ID, esc->currentP(), config.refresh.curr, esc);
        addSensor(sensorP);
        //sensorP = new Sensord(SBEC_POWER_FIRST_ID, esc->becVoltageP(), esc->becCurrentP(), config.refresh.volt, esc);
        //addSensor(sensorP);
        sensorP = new Sensord(TEMP1_ID, esc->tempFetP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensord(TEMP2_ID, esc->tempBecP(), config.refresh.temp, esc);
        addSensor(sensorP);
        sensorP = new Sensord(VFAS_ID, esc->cellVoltageP(), config.refresh.volt, esc);
        addSensor(sensorP);
    }
    if (config.gps == true)
    {
        Sensord *sensorP;
        Bn220 *gps;
        GPS_SERIAL.begin(GPS_BAUD_RATE);
        GPS_SERIAL.setTimeout(BN220_TIMEOUT);
        gps = new Bn220(GPS_SERIAL);
        sensorP = new Sensord(GPS_LONG_BP_ID, gps->lonP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_LONG_AP_ID, gps->lonP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_LONG_EW_ID, gps->lonP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_LAT_BP_ID, gps->latP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_LAT_AP_ID, gps->latP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_LAT_NS_ID, gps->latP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_ALT_BP_ID, gps->altP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_ALT_AP_ID, gps->altP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_SPEED_BP_ID, gps->spdP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_SPEED_AP_ID, gps->spdP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_COURS_BP_ID, gps->cogP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_COURS_AP_ID, gps->cogP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_YEAR_ID, gps->dateP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_DAY_MONTH_ID, gps->dateP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_HOUR_MIN_ID, gps->timeP(), 10, gps);
        addSensor(sensorP);
        sensorP = new Sensord(GPS_SEC_ID, gps->timeP(), 10, gps);
        addSensor(sensorP);
    }
    if (config.airspeed == true)
    {
        Sensord *sensorP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorP = new Sensord(VARIO_ID, pressure->valueP(), config.refresh.volt, pressure);
        addSensor(sensorP);
    }
    if (config.voltage1 == true)
    {
        Sensord *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt));
        sensorP = new Sensord(VOLTS_BP_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
        sensorP = new Sensord(VOLTS_AP_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
    }
    /*if (config.voltage2 == true)
    {
        Sensord *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt));
        sensorP = new Sensord(A4_FIRST_ID, voltage->valueP(), config.refresh.volt, voltage);
        addSensor(sensorP);
    }*/
    if (config.current == true)
    {
        Sensord *sensorP;
        Voltage *current;
        current = new Voltage(PIN_CURRENT, ALPHA(config.average.curr));
        sensorP = new Sensord(CURRENT_ID, current->valueP(), config.refresh.curr, current);
        addSensor(sensorP);
    }
    if (config.ntc1 == true)
    {
        Sensord *sensorP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorP = new Sensord(TEMP1_ID, ntc->valueP(), config.refresh.temp, ntc);
        addSensor(sensorP);
    }
    if (config.ntc2 == true)
    {
        Sensord *sensorP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorP = new Sensord(TEMP2_ID, ntc->valueP(), config.refresh.temp, ntc);
        addSensor(sensorP);
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        Sensord *sensorP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), 10);
        bmp->begin();
        sensorP = new Sensord(TEMP1_ID, bmp->temperatureP(), config.refresh.temp, bmp);
        addSensor(sensorP);
        sensorP = new Sensord(BARO_ALT_BP_ID, bmp->altitudeP(), 10, bmp);
        addSensor(sensorP);
        sensorP = new Sensord(BARO_ALT_AP_ID, bmp->altitudeP(), 10, bmp);
        addSensor(sensorP);
    }
}