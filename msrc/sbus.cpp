#include "sbus.h"

Sbus::Sbus(AbstractSerial &serial) : serial_(serial)
{
}

Sbus::~Sbus()
{
    deleteSensors();
}

SensorSbus *Sbus::sensorSbusP[32] = {NULL};

const uint8_t Sbus::slotId[32] = {0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                                  0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                                  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                                  0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB};

uint8_t Sbus::telemetryPacket = 0;

uint32_t Sbus::ts2 = 0;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

void Sbus::TIMER_COMPA_handler()
{
    uint8_t ts = TCNT2;
    static uint8_t cont = 0;
#ifdef DEBUG_SBUS_MS
    static uint16_t msprev = 0;
    uint16_t ms = micros();
    if (cont == 0)
    {
        DEBUG_PRINT(" ");
        DEBUG_PRINT(micros() - ts2);
    }
    else
    {
        DEBUG_PRINT(" ");
        DEBUG_PRINT((uint16_t)(ms - msprev));
    }
    msprev = ms;
#endif
#ifdef DEBUG
    DEBUG_PRINT(" ");
    DEBUG_PRINT(telemetryPacket * 8 + cont);
#endif
    if (sensorSbusP[telemetryPacket * 8 + cont] != NULL)
        sendSlot(telemetryPacket * 8 + cont);
    if (cont < 7)
    {
        OCR2A = ts + (uint8_t)(700 * US_TO_COMP(256)); // next slot  540us
        cont++;
    }
    else
    {
        TIMSK2 &= ~_BV(OCIE2A); // DISABLE TIMER2 OCRA INTERRUPT
        cont = 0;
        digitalWrite(LED_BUILTIN, LOW);
    }
}

#endif

#if defined(__AVR_ATmega32U4__)

void Sbus::TIMER_COMPA_handler()
{
    uint16_t ts = TCNT3;
    static uint8_t cont = 0;
#ifdef DEBUG_SBUS_MS
    static uint16_t msprev = 0;
    uint16_t ms = micros();
    if (cont == 0)
    {
        DEBUG_PRINT(" ");
        DEBUG_PRINT(micros() - ts2);
    }
    else
    {
        DEBUG_PRINT(" ");
        DEBUG_PRINT((uint16_t)(ms - msprev));
    }
    msprev = ms;
#endif
#ifdef DEBUG
    DEBUG_PRINT(" ");
    DEBUG_PRINT(telemetryPacket * 8 + cont);
#endif

    if (sensorSbusP[telemetryPacket * 8 + cont] != NULL)
        sendSlot(telemetryPacket * 8 + cont);
    if (cont < 7)
    {
        OCR3A = ts + (uint16_t)(700 * US_TO_COMP(1)); // next slot 700us
        cont++;
    }
    else
    {
        TIMSK3 &= ~_BV(OCIE3A); // DISABLE TIMER2 OCRA INTERRUPT
        cont = 0;
        digitalWrite(LED_BUILTIN, LOW);
    }
}

#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
void Sbus::FTM0_IRQ_handler()
{
    if (FTM0_C0SC & FTM_CSC_CHF) // CH0 INTERRUPT
    {
        uint16_t ts = FTM0_CNT;
        static uint8_t cont = 0;
#ifdef DEBUG_SBUS_MS
        static uint16_t msprev = 0;
        uint16_t ms = micros();
        if (cont == 0)
        {
            DEBUG_PRINT(" ");
            DEBUG_PRINT(micros() - ts2);
        }
        else
        {
            DEBUG_PRINT(" ");
            DEBUG_PRINT((uint16_t)(ms - msprev));
        }
        msprev = ms;
#endif
#ifdef DEBUG
        DEBUG_PRINT(" ");
        DEBUG_PRINT(telemetryPacket * 8 + cont);
#endif
        if (sensorSbusP[telemetryPacket * 8 + cont] != NULL)
            sendSlot(telemetryPacket * 8 + cont);
        if (cont < 7)
        {
            FTM0_C0V = ts + (uint16_t)(700 * US_TO_COMP(128)); // next slot 700us
            cont++;
        }
        else
        {
            FTM0_C0SC &= ~FTM_CSC_CHIE; // DISABLE CH0 INTERRUPT
            cont = 0;
            digitalWrite(LED_BUILTIN, LOW);
        }
        FTM0_C0SC |= FTM_CSC_CHF; // CLEAR FLAG CH2
    }
}

#endif

void Sbus::sendSlot(uint8_t number)
{
    SMARTPORT_FRSKY_SBUS_SERIAL.write(slotId[number]);
    uint16_t val = sensorSbusP[number]->valueFormatted();
    SMARTPORT_FRSKY_SBUS_SERIAL.write(val);
    SMARTPORT_FRSKY_SBUS_SERIAL.write(val >> 8);
#ifdef DEBUG
    DEBUG_PRINT(":");
    DEBUG_PRINT_HEX(slotId[number]);
    DEBUG_PRINT(":");
    DEBUG_PRINT_HEX(sensorSbusP[number]->valueFormatted());
#endif
}

void Sbus::begin()
{
    SMARTPORT_FRSKY_SBUS_SERIAL.begin(100000, SERIAL_8E2_RXINV_TXINV | SERIAL_HALF_DUP);
    //SMARTPORT_FRSKY_SBUS_SERIAL.begin(100000, SERIAL_8E2);
    SMARTPORT_FRSKY_SBUS_SERIAL.setTimeout(SBUS_SERIAL_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)
    // TIMER 2 - shared with softserial
    TIMER2_COMPA_handlerP = TIMER_COMPA_handler;
    TCCR2B = _BV(CS22) | _BV(CS21); // SCALER 256
    TCCR2A = 0;
#endif

#if defined(__AVR_ATmega32U4__)
    // TIMER 3
    TIMER3_COMPA_handlerP = TIMER_COMPA_handler;
    TCCR3B = _BV(CS30); // SCALER 1
    TCCR3A = 0;
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    FTM0_IRQ_handlerP = FTM0_IRQ_handler;
    FTM0_SC = 0;
    delayMicroseconds(1);
    FTM0_CNT = 0;
    SIM_SCGC6 |= SIM_SCGC6_FTM0; // ENABLE CLOCK
    FTM0_SC = FTM_SC_PS(7);      // PRESCALER 128    FTM_SC_PS(5) - PRESCALER 32
    FTM0_SC |= FTM_SC_CLKS(1);   // ENABLE COUNTER
    FTM0_C0SC = 0;               // DISABLE CHANNEL
    delayMicroseconds(1);
    FTM0_C0SC = FTM_CSC_MSA; // SOFTWARE COMPARE
    NVIC_ENABLE_IRQ(IRQ_FTM0);
#endif
}

void Sbus::addSensor(uint8_t number, SensorSbus *newSensorSbusP)
{
    sensorSbusP[number] = newSensorSbusP;
}

void Sbus::deleteSensors()
{
    for (uint8_t i = 0; i < 32; i++)
    {
        if (sensorSbusP[i] != NULL)
            delete sensorSbusP[i];
    }
}

void Sbus::sendPacket()
{
    digitalWrite(LED_BUILTIN, HIGH);
    uint16_t ts = SMARTPORT_FRSKY_SBUS_SERIAL.timestamp();
#ifdef DEBUG
    DEBUG_PRINT("\nTP");
    DEBUG_PRINT(telemetryPacket);
    DEBUG_PRINT(": ");
#endif
#ifdef DEBUG_SBUS_MS
    DEBUG_PRINTLN();
    DEBUG_PRINT(ts);
    DEBUG_PRINT("+");
    ts2 = micros();
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)
    // configure timer
    OCR2A = TCNT2 + (2000 - ts) * US_TO_COMP(256); // complete 2ms
    TIFR2 |= _BV(OCF2A);                           // CLEAR TIMER2 OCRA CAPTURE FLAG
    TIMSK2 |= _BV(OCIE2A);                         // ENABLE TIMER2 OCRA INTERRUPT
#endif

#if defined(__AVR_ATmega32U4__)
    // configure timer
    OCR3A = TCNT3 + (2000 - ts) * US_TO_COMP(1); // complete 2ms
    TIFR3 |= _BV(OCF3A);                         // CLEAR TIMER2 OCRA CAPTURE FLAG
    TIMSK3 |= _BV(OCIE3A);                       // ENABLE TIMER2 OCRA INTERRUPT
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    FTM0_C0V = FTM0_CNT + (2000 - ts) * US_TO_COMP(128);
    FTM0_C0SC |= FTM_CSC_CHF;                     // CLEAR FLAG
    FTM0_C0SC |= FTM_CSC_CHIE;                    // ENABLE INTERRUPT
#endif
}

void Sbus::update()
{
    uint8_t status = SBUS_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)(millis() - ts) > 100)
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
    uint8_t lenght = SMARTPORT_FRSKY_SBUS_SERIAL.availableTimeout();
    uint8_t buff[lenght];
    SMARTPORT_FRSKY_SBUS_SERIAL.readBytes(buff, lenght);
#ifdef DEBUG_PACKET
    if (lenght)
    {
        DEBUG_PRINT("\n<");
        for (uint8_t i = 0; i < lenght; i++)
        {
            DEBUG_PRINT_HEX(buff[i]);
            DEBUG_PRINT(" ");
        }
    }
#endif
    if (lenght == SBUS_PACKET_LENGHT)
    {
        //uint8_t buff[SBUS_PACKET_LENGHT];
        //SMARTPORT_FRSKY_SBUS_SERIAL.readBytes(buff, SBUS_PACKET_LENGHT);
        if (buff[0] == 0x0F)
        {
            // SBUS
            if (buff[24] == 0)
            {
                if (!mute)
                {
                    status = SBUS_SEND;
                    telemetryPacket++;
                    if (telemetryPacket == 4)
                        telemetryPacket = 0;
                }
                mute = !mute;
            }
            // SBUS2
            else if (buff[24] == 0x04 || buff[24] == 0x14 || buff[24] == 0x24 || buff[24] == 0x34)
            {
                telemetryPacket = buff[24] >> 4;
                status = SBUS_SEND;
            }
        }
    }
#endif
    if (status == SBUS_SEND)
    {
        sendPacket();
    }

    // update sensor
    static uint8_t cont = 0;
    if (sensorSbusP[cont])
    {
        if (sensorSbusP[cont]->valueP())
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
        esc->begin();
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_HW_V4)
    {
        SensorSbus *sensorSbusP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, esc->consumptionP(), esc);
        addSensor(10, sensorSbusP);
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
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, esc->consumptionP(), esc);
        addSensor(10, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->rippleVoltageP(), esc);
        //addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->becCurrentP(), esc);
        addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->becVoltageP(), esc);
        addSensor(12, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, NULL, NULL);
        addSensor(13, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->temperatureP(), esc);
        addSensor(6, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_KONTRONIK)
    {
        SensorSbus *sensorSbusP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, esc->consumptionP(), esc);
        addSensor(10, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->becCurrentP(), esc);
        addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->becVoltageP(), esc);
        addSensor(12, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, NULL, NULL);
        addSensor(13, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempFetP(), esc);
        addSensor(6, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempFetP(), esc);
        addSensor(7, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_APD_F)
    {
        SensorSbus *sensorSbusP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, esc->consumptionP(), esc);
        addSensor(10, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempP(), esc);
        addSensor(6, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.protocol == PROTOCOL_APD_HV)
    {
        SensorSbus *sensorSbusP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        //PwmOut pwmOut;
        //pwmOut.setRpmP(esc->rpmP());
        sensorSbusP = new SensorSbus(FASST_RPM, esc->rpmP(), esc);
        addSensor(1, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, esc->currentP(), esc);
        addSensor(8, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->voltageP(), esc);
        addSensor(9, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, esc->consumptionP(), esc);
        addSensor(10, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_TEMP, esc->tempP(), esc);
        addSensor(6, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_POWER_VOLT, esc->cellVoltageP(), esc);
        //addSensor(12, sensorSbusP);
    }
    if (config.gps == true)
    {
        SensorSbus *sensorSbusP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        sensorSbusP = new SensorSbus(FASST_GPS_SPEED, gps->spdP(), gps);
        addSensor(16, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_ALTITUDE, gps->altP(), gps);
        addSensor(17, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_TIME, gps->timeP(), gps);
        addSensor(18, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_VARIO_SPEED, NULL, NULL);
        addSensor(19, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LATITUDE1, gps->latP(), gps);
        addSensor(20, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LATITUDE2, gps->latP(), gps);
        addSensor(21, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LONGITUDE1, gps->lonP(), gps);
        addSensor(22, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_GPS_LONGITUDE2, gps->lonP(), gps);
        addSensor(23, sensorSbusP);
        //sensorSbusP = new SensorSbus(FASST_VARIO_ALT, gps->varioP(), gps);
        //addSensor(5, sensorSbusP);
    }
    if (config.airspeed == true)
    {
        SensorSbus *sensorSbusP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        sensorSbusP = new SensorSbus(FASST_VARIO_SPEED, pressure->valueP(), pressure);
        addSensor(4, sensorSbusP);
        if (config.deviceI2C1Type == I2C_BMP280)
        {
            sensorSbusP = new SensorSbus(FASST_VARIO_ALT, NULL, NULL);
            addSensor(5, sensorSbusP);
        }
    }
    if (config.voltage1 == true)
    {
        SensorSbus *sensorSbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        sensorSbusP = new SensorSbus(FASST_VOLT_V1, voltage->valueP(), voltage);
        addSensor(2, sensorSbusP);

        if (config.voltage2 == false)
        {
            sensorSbusP = new SensorSbus(FASST_VOLT_V2, NULL, NULL);
            addSensor(3, sensorSbusP);
        }
    }
    if (config.voltage2 == true)
    {
        SensorSbus *sensorSbusP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt), VOLTAGE2_MULTIPLIER);
        sensorSbusP = new SensorSbus(FASST_VOLT_V2, voltage->valueP(), voltage);
        addSensor(3, sensorSbusP);
        if (config.voltage1 == false)
        {
            sensorSbusP = new SensorSbus(FASST_VOLT_V1, NULL, NULL);
            addSensor(2, sensorSbusP);
        }
    }
    if (config.current == true)
    {
        SensorSbus *sensorSbusP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        sensorSbusP = new SensorSbus(FASST_POWER_CURR, current->valueP(), current);
        addSensor(11, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_VOLT, NULL, NULL);
        addSensor(12, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_POWER_CONS, current->consumptionP(), current);
        addSensor(13, sensorSbusP);
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
        sensorSbusP = new SensorSbus(FASST_VARIO_SPEED, bmp->varioP(), bmp);
        addSensor(4, sensorSbusP);
        sensorSbusP = new SensorSbus(FASST_VARIO_ALT, bmp->altitudeP(), bmp);
        addSensor(5, sensorSbusP);
        if (config.airspeed == true)
        {
            sensorSbusP = new SensorSbus(FASST_VARIO_SPEED, NULL, NULL);
            addSensor(4, sensorSbusP);
        }
    }
}
