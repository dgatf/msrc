#include "sbus.h"

Sbus::Sbus(Stream &serial) : serial_(serial)
{
}

Sbus::~Sbus()
{
    deleteSensors();
}

void Sbus::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Sbus::addSensor(uint8_t number, SensorSbus *newSensorSbusP)
{
    sensorSbusP[number] = newSensorSbusP;
}

void Sbus::deleteSensors()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (sensorSbusP[i] != NULL)
            delete sensorSbusP[i];
    }
}

void Sbus::sendPacket(uint8_t telemetryPacket)
{
    for (uint8_t i = 0; i < telemetryPacket * 8; i++)
    {
        if (sensorSbusP[i] != NULL)
            sendSlot(i);
        delay(200);
    }
}

void Sbus::sendSlot(uint8_t number)
{
}

void Sbus::update()
{
    uint8_t status = SBUS_WAIT;
    static uint8_t telemetryPacket = 0;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)millis() - ts > 100)
    {
        if (!mute)
        {
            status = SBUS_SEND;
            telemetryPacket++;
        }
        mute = !mute;
        ts = millis();
        if (telemetryPacket == 4)
            telemetryPacket = 0;
    }
#else
    if (serial_.available())
    {
        uint8_t buff[25];
        uint8_t result = serial_.readBytes(buff, 25);
        if (result == 25 && buff[24] == 0x0F && buff[24] == 0x04)
        {
            if (!mute)
            {
                status = SBUS_SEND;
                telemetryPacket++;
            }
            mute = !mute;
            if (telemetryPacket == 4)
                telemetryPacket = 0;
        }

#endif
    if (status == SBUS_SEND)
    {
        sendPacket(telemetryPacket);
    }
}

// update sensor
static uint8_t cont = 0;
if (sensorSbusP[cont])
{
    sensorSbusP[cont]->update();
}
cont++;
if (cont == 32)
    cont = 0;
}

void Sbus::setConfig(Config &config)
{
    deleteSensors();
    if (config.protocol == PROTOCOL_PWM)
    {
        SensorSbus *sensorSbusP;
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        SensorSbus *sensorSbusP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        SensorSbus *sensorSbusP;
        EscHW4 *esc;
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), config.protocol - PROTOCOL_HW_V4_LV);
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempFetP(), esc);
        addSensor(6, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempBecP(), esc);
        addSensor(7, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_CASTLE)
    {
        SensorSbus *sensorSbusP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->rippleVoltageP(), esc);
        //addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->becCurrentP(), esc);
        addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->becVoltageP(), esc);
        addSensor(12, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->temperatureP(), esc);
        addSensor(6, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        SensorSbus *sensorSbusP;
        EscKontronik *esc;
        ESC_SERIAL.begin(115200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->becCurrentP(), esc);
        addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->becVoltageP(), esc);
        addSensor(12, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempFetP(), esc);
        addSensor(6, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempFetP(), esc);
        addSensor(7, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.gps == true)
    {
        SensorSbus *sensorSbusP;
        Bn220 *gps;
        GPS_SERIAL.begin(GPS_BAUD_RATE);
        GPS_SERIAL.setTimeout(BN220_TIMEOUT);
        gps = new Bn220(GPS_SERIAL);
        sensorSbusP = new SensorSbus(FASST_GPS_SPEED, gps->spdP(), gps);
        addSensor(16, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_ALTITUDE, gps->altP(), gps);
        addSensor(17, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_COG, gps->cogP(), gps);
        addSensor(18, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LATITUDE1, gps->latP(), gps);
        addSensor(19, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LATITUDE2, gps->latP(), gps);
        addSensor(20, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LATITUDE3, gps->latP(), gps);
        addSensor(21, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LONGITUDE1, gps->lonP(), gps);
        addSensor(22, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LONGITUDE2, gps->lonP(), gps);
        addSensor(23, sensorSbusP);
    }
    if (config.airspeed == true)
    {
        SensorSbus *sensorSbusP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorSbusP = new SensorSbus(FASST_VARIO_SPEED, pressure->valueP(), pressure);
        addSensor(4, sensorSbusP);
    }
    if (config.voltage1 == true)
    {
        SensorSbus *sensorSbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt));
        sensorSbusP = new SensorSbus(FASST_VOLT_V1, voltage->valueP(), voltage);
        addSensor(2, sensorSbusP);
    }
    if (config.voltage2 == true)
    {
        SensorSbus *sensorSbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt));
        sensorSbusP = new SensorSbus(FASST_VOLT_V2, voltage->valueP(), voltage);
        addSensor(3, sensorSbusP);
    }
    if (config.current == true)
    {
        SensorSbus *sensorSbusP;
        Voltage *current;
        current = new Voltage(PIN_CURRENT, ALPHA(config.average.curr));
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, current->valueP(), current);
        addSensor(8, sensorSbusP);
    }
    if (config.ntc1 == true)
    {
        SensorSbus *sensorSbusP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        sensorSbusP = new SensorSbus(FASST_TEMP, ntc->valueP(), ntc);
        addSensor(6, sensorSbusP);
    }
    if (config.ntc2 == true)
    {
        SensorSbus *sensorSbusP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        sensorSbusP = new SensorSbus(FASST_TEMP, ntc->valueP(), ntc);
        addSensor(7, sensorSbusP);
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        SensorSbus *sensorSbusP;
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), 10);
        bmp->begin();
        //sensorSbusP = new SensorSbus(TEMP1_ID, bmp->temperatureP(), bmp);
        sensorSbusP = new SensorSbus(FASST_VARIO_ALT, bmp->altitudeP(), bmp);
        addSensor(5, sensorSbusP);
    }
}