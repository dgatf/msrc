#include "multiplex.h"

Multiplex::Multiplex(AbstractSerial &serial) : serial_(serial)
{
}

Multiplex::~Multiplex()
{
    deleteSensors();
}

void Multiplex::begin()
{
    serial_.begin(38400, SERIAL__8N1 | SERIAL__HALF_DUP);
    serial_.setTimeout(MULTIPLEX_SERIAL_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Multiplex::addSensor(SensorMultiplex *newSensorMultiplexP)
{
    static uint8_t cont = 1;
    if (cont < 16)
    {
        sensorMultiplexP[cont] = newSensorMultiplexP;
        cont++;
    }
}

void Multiplex::deleteSensors()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (sensorMultiplexP[i] != NULL)
            delete sensorMultiplexP[i];
    }
}

void Multiplex::sendPacket(uint8_t address)
{
    if (!sensorMultiplexP[address])
        return;
    digitalWrite(LED_BUILTIN, HIGH);
    serial_.write(address << 4 | sensorMultiplexP[address]->dataId());
    int16_t value = sensorMultiplexP[address]->valueFormatted();
    serial_.write(value);
    serial_.write(value >> 8);
    digitalWrite(LED_BUILTIN, LOW);
#ifdef DEBUG
    DEBUG_PRINT_HEX((uint8_t)(address << 4 | sensorMultiplexP[address]->dataId()));
    DEBUG_PRINT(" ");
    DEBUG_PRINT_HEX((uint16_t)value);
    DEBUG_PRINTLN();
#endif
}

void Multiplex::update()
{
    uint8_t status = MULTIPLEX_WAIT;
    uint8_t address = 0;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)(millis() - ts) > 100)
    {
        status = MULTIPLEX_SEND;
        ts = millis();
    }
#else
    if (serial_.availableTimeout() == 1)
    {
        address = serial_.read();
        if (address < 16)
        {
            status = MULTIPLEX_SEND;
        }
    }
#endif
    if (status == MULTIPLEX_SEND)
    {
        sendPacket(address);
    }

    // update sensor
    static uint8_t cont = 0;
    if (sensorMultiplexP[cont])
    {
        sensorMultiplexP[cont]->update();
    }
    cont++;
    if (cont == 16)
        cont = 0;
}

void Multiplex::setConfig(Config &config)
{
    deleteSensors();
    if (ESC_PROTOCOL == PROTOCOL_PWM)
    {
        SensorMultiplex *sensorMultiplexP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V3)
    {
        SensorMultiplex *sensorMultiplexP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        SensorMultiplex *sensorMultiplexP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->currentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->voltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempFetP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempBecP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->cellVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CONSUMPTION, esc->consumptionP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        SensorMultiplex *sensorMultiplexP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->currentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->voltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->rippleVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->becCurrentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->becVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->temperatureP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->cellVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CONSUMPTION, esc->consumptionP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        SensorMultiplex *sensorMultiplexP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->currentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->voltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->becCurrentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->becVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempFetP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempFetP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->cellVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CONSUMPTION, esc->consumptionP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        SensorMultiplex *sensorMultiplexP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->currentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->voltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CONSUMPTION, esc->consumptionP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->cellVoltageP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        SensorMultiplex *sensorMultiplexP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->currentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->voltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CONSUMPTION, esc->consumptionP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->cellVoltageP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (config.gps == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_SPEED, gps->spdP(), gps);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_ALTITUDE, gps->altP(), gps);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VARIO, gps->varioP(), gps);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_DISTANCE, gps->distP(), gps);
        addSensor(sensorMultiplexP);
    }
    if (config.airspeed == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorMultiplexP = new SensorMultiplex(FHSS_SPEED, pressure->valueP(), pressure);
        addSensor(sensorMultiplexP);
    }
    if (config.voltage1 == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, voltage->valueP(), voltage);
        addSensor(sensorMultiplexP);
    }
    if (config.voltage2 == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt), VOLTAGE2_MULTIPLIER);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, voltage->valueP(), voltage);
        addSensor(sensorMultiplexP);
    }
    if (config.current == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, current->valueP(), current);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CONSUMPTION, current->consumptionP(), current);
        addSensor(sensorMultiplexP);
    }
    if (config.ntc1 == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, ntc->valueP(), ntc);
        addSensor(sensorMultiplexP);
    }
    if (config.ntc2 == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, ntc->valueP(), ntc);
        addSensor(sensorMultiplexP);
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        SensorMultiplex *sensorMultiplexP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, bmp->temperatureP(), bmp);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_ALTITUDE, bmp->altitudeP(), bmp);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VARIO, bmp->varioP(), bmp);
        addSensor(sensorMultiplexP);
    }
    if (config.deviceI2C1Type == I2C_MS5611)
    {
        SensorMultiplex *sensorMultiplexP;
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, bmp->temperatureP(), bmp);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_ALTITUDE, bmp->altitudeP(), bmp);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VARIO, bmp->varioP(), bmp);
        addSensor(sensorMultiplexP);
    }
}
