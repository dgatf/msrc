#include "multiplex.h"

Multiplex::Multiplex(Stream &serial) : serial_(serial)
{
}

Multiplex::~Multiplex()
{
    deleteSensors();
}

void Multiplex::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Multiplex::addSensor(SensorMultiplex *newSensorMultiplexP)
{
    static uint8_t cont = 1;
    sensorMultiplexP[cont] = newSensorMultiplexP;
    cont++;
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
    bool isNegative = false;
    serial_.write(address << 4 | sensorMultiplexP[address]->dataId());
    int16_t value = sensorMultiplexP[address]->valueFormatted();
    if (value < 0)
    {
        value *= -1;
        isNegative = true;
    }
    if (value > 0x3FF7)
        value = 0x3FF7;
    value <<= 1;
    if (isNegative)
        value |= 1 << 15;
    serial_.write(value);
#ifdef DEBUG
    DEBUG_SERIAL.print(address << 4 | sensorMultiplexP[address]->dataId(), HEX);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(value, HEX);
#endif
}

void Multiplex::update()
{
    static uint16_t serialTs = 0;
    static uint8_t serialCount = 0;
    uint8_t status = MULTIPLEX_WAIT;
    uint8_t address = 0;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)millis() - ts > 100)
    {
        if (!mute)
        {
            status = MULTIPLEX_SEND;
            telemetryPacket++;
        }
        mute = !mute;
        ts = millis();
    }
#else
    if (serial_.available() > serialCount)
    {
        serialCount = serial_.available();
        serialTs = micros();
    }
    
    if (((uint16_t)micros() - serialTs > MULTIPLEX_SERIAL_TIMEOUT) && serialCount > 0)
    {
        if (serialCount == 1)
        {
            address = serial_.read();
            if (address <= 16)
            {
                status = MULTIPLEX_SEND;
            }
        }
        while (serial_.available())
            serial_.read();
        serialCount = 0;
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
    if (config.protocol == PROTOCOL_PWM)
    {
        SensorMultiplex *sensorMultiplexP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        SensorMultiplex *sensorMultiplexP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        SensorMultiplex *sensorMultiplexP;
        EscHW4 *esc;
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), config.protocol - PROTOCOL_HW_V4_LV);
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
        //sensorMultiplexP = new SensorMultiplex(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorMultiplexP);
    }
    if (config.protocol == PROTOCOL_CASTLE)
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
        //sensorMultiplexP = new SensorMultiplex(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(sensorMultiplexP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        SensorMultiplex *sensorMultiplexP;
        EscKontronik *esc;
        ESC_SERIAL.begin(115200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorMultiplexP = new SensorMultiplex(FHSS_RPM, esc->rpmP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->currentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, esc->voltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, esc->becCurrentP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FASST_POWER_VOLT, esc->becVoltageP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempFetP(), esc);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_TEMP, esc->tempFetP(), esc);
        addSensor(sensorMultiplexP);
        //sensorMultiplexP = new SensorMultiplex(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(sensorMultiplexP);
    }
    if (config.gps == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Bn220 *gps;
        GPS_SERIAL.begin(GPS_BAUD_RATE);
        GPS_SERIAL.setTimeout(BN220_TIMEOUT);
        gps = new Bn220(GPS_SERIAL);
        sensorMultiplexP = new SensorMultiplex(FHSS_SPEED, gps->spdP(), gps);
        addSensor(sensorMultiplexP);
        sensorMultiplexP = new SensorMultiplex(FHSS_ALTITUDE, gps->altP(), gps);
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
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt));
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, voltage->valueP(), voltage);
        addSensor(sensorMultiplexP);
    }
    if (config.voltage2 == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt));
        sensorMultiplexP = new SensorMultiplex(FHSS_VOLTAGE, voltage->valueP(), voltage);
        addSensor(sensorMultiplexP);
    }
    if (config.current == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Voltage *current;
        current = new Voltage(PIN_CURRENT, ALPHA(config.average.curr));
        sensorMultiplexP = new SensorMultiplex(FHSS_CURRENT, current->valueP(), current);
        addSensor(sensorMultiplexP);
    }
    if (config.ntc1 == true)
    {
        SensorMultiplex *sensorMultiplexP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorMultiplexP = new SensorMultiplex(FASST_TEMP, ntc->valueP(), ntc);
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
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), 10);
        bmp->begin();
        //sensorMultiplexP = new SensorMultiplex(TEMP1_ID, bmp->temperatureP(), bmp);
        sensorMultiplexP = new SensorMultiplex(FHSS_ALTITUDE, bmp->altitudeP(), bmp);
        addSensor(sensorMultiplexP);
    }
}
