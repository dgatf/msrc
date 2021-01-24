#include "msrc.h"

#if !defined(__AVR_ATmega328P__ ) && !defined(__AVR_ATmega328PB__ ) && !defined(__AVR_ATmega2560__ ) && !defined(__AVR_ATmega32U4__)
#warning "MCU not supported"
#endif

// ISR handlers

#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)
void (*TIMER1_CAPT_handlerP)() = NULL;
ISR(TIMER1_CAPT_vect)
{
    if (TIMER1_CAPT_handlerP)
        TIMER1_CAPT_handlerP();
}
void (*TIMER1_COMPB_handlerP)() = NULL;
ISR(TIMER1_COMPB_vect)
{
    if (TIMER1_COMPB_handlerP)
        TIMER1_COMPB_handlerP();
}
void (*TIMER1_OVF_handlerP)() = NULL;
ISR(TIMER1_OVF_vect)
{
    if (TIMER1_OVF_handlerP)
        TIMER1_OVF_handlerP();
}
void (*INT0_handlerP)() = NULL;
ISR(INT0_vect)
{
    if (INT0_handlerP)
        INT0_handlerP();
}
void (*TIMER2_COMPA_handlerP)() = NULL;
ISR(TIMER2_COMPA_vect)
{
    if (TIMER2_COMPA_handlerP)
        TIMER2_COMPA_handlerP();
}
#endif

#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
void (*TIMER1_CAPT_handlerP)() = NULL;
ISR(TIMER1_CAPT_vect)
{
    if (TIMER1_CAPT_handlerP)
        TIMER1_CAPT_handlerP();
}
void (*TIMER1_COMPB_handlerP)() = NULL;
ISR(TIMER1_COMPB_vect)
{
    if (TIMER1_COMPB_handlerP)
        TIMER1_COMPB_handlerP();
}
void (*TIMER1_OVF_handlerP)() = NULL;
ISR(TIMER1_OVF_vect)
{
    if (TIMER1_OVF_handlerP)
        TIMER1_OVF_handlerP();
}
void (*TIMER2_COMPA_handlerP)() = NULL;
ISR(TIMER2_COMPA_vect)
{
    if (TIMER2_COMPA_handlerP)
        TIMER2_COMPA_handlerP();
}
void (*TIMER4_COMPB_handlerP)() = NULL;
ISR(TIMER4_COMPB_vect)
{
    if (TIMER4_COMPB_handlerP)
        TIMER4_COMPB_handlerP();
}
void (*TIMER4_CAPT_handlerP)() = NULL;
ISR(TIMER4_CAPT_vect)
{
    if (TIMER4_CAPT_handlerP)
        TIMER4_CAPT_handlerP();
}
#endif

#if defined(__AVR_ATmega2560__)
void (*TIMER4_CAPT_handlerP)() = NULL;
ISR(TIMER4_CAPT_vect)
{
    if (TIMER4_CAPT_handlerP)
        TIMER4_CAPT_handlerP();
}
void (*TIMER4_COMPB_handlerP)() = NULL;
ISR(TIMER4_COMPB_vect)
{
    if (TIMER4_COMPB_handlerP)
        TIMER4_COMPB_handlerP();
}
void (*TIMER4_OVF_handlerP)() = NULL;
ISR(TIMER4_OVF_vect)
{
    if (TIMER4_OVF_handlerP)
        TIMER4_OVF_handlerP();
}
void (*TIMER5_COMPB_handlerP)() = NULL;
ISR(TIMER5_COMPB_vect)
{
    if (TIMER5_COMPB_handlerP)
        TIMER5_COMPB_handlerP();
}
void (*TIMER5_COMPC_handlerP)() = NULL;
ISR(TIMER5_COMPC_vect)
{
    if (TIMER5_COMPC_handlerP)
        TIMER5_COMPC_handlerP();
}
void (*TIMER5_CAPT_handlerP)() = NULL;
ISR(TIMER5_CAPT_vect)
{
    if (TIMER5_CAPT_handlerP)
        TIMER5_CAPT_handlerP();
}
#endif

#if defined(__AVR_ATmega32U4__)
void (*TIMER1_CAPT_handlerP)() = NULL;
ISR(TIMER1_CAPT_vect)
{
    if (TIMER1_CAPT_handlerP)
        TIMER1_CAPT_handlerP();
}
void (*TIMER1_COMPB_handlerP)() = NULL;
ISR(TIMER1_COMPB_vect)
{
    if (TIMER1_COMPB_handlerP)
        TIMER1_COMPB_handlerP();
}
void (*TIMER1_OVF_handlerP)() = NULL;
ISR(TIMER1_OVF_vect)
{
    if (TIMER1_OVF_handlerP)
        TIMER1_OVF_handlerP();
}
void (*TIMER1_COMPC_handlerP)() = NULL;
ISR(TIMER1_COMPC_vect)
{
    if (TIMER1_COMPC_handlerP)
        TIMER1_COMPC_handlerP();
}
void (*TIMER3_COMPB_handlerP)() = NULL;
ISR(TIMER3_COMPB_vect)
{
    if (TIMER3_COMPB_handlerP)
        TIMER3_COMPB_handlerP();
}
void (*TIMER3_CAPT_handlerP)() = NULL;
ISR(TIMER3_CAPT_vect)
{
    if (TIMER3_CAPT_handlerP)
        TIMER3_CAPT_handlerP();
}
void (*TIMER3_OVF_handlerP)() = NULL;
ISR(TIMER3_OVF_vect)
{
    if (TIMER3_OVF_handlerP)
        TIMER3_OVF_handlerP();
}
#endif

uint8_t calcAlpha(uint8_t elements)
{
    return round((2.0F / (elements + 1)) * 100);
}

void setPwmOut(bool pwmOut)
{
    noInterrupts();
    if (pwmOut)
    {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
        // TIMER1: MODE 15 (TOP OCRA), SCALER 8. OC1B, PB2, PIN 10
        DDRB |= _BV(DDB2);
        TCCR1A = _BV(WGM11) | _BV(WGM10);
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
#endif
#if defined(__AVR_ATmega2560__)
        // TIMER4: MODE 15 (TOP OCRA), SCALER 8. OC4B, PH4, PIN 7
        DDRH |= _BV(DDH4);
        TCCR4A = _BV(WGM41) | _BV(WGM40);
        TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);
#endif
#if defined(__AVR_ATmega32U4__)
        // TIMER1: MODE 15 (TOP OCRA), SCALER 8. OC1B, PB6
        DDRB |= _BV(DDB6);
        TCCR1A = _BV(WGM11) | _BV(WGM10);
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
#endif
    }
    else
    {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
        TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
#endif
#if defined(__AVR_ATmega2560__)
        TCCR4A &= ~_BV(COM4A1) & ~_BV(COM4B1);
#endif
    }
    interrupts();
}

void updatePwmOut()
{
    static float rpm = 0;
    if (rpmPwmoutP == NULL)
        return;
    if (*rpmPwmoutP != rpm)
    {
        rpm = *rpmPwmoutP;
        noInterrupts();
        if (rpm >= 2000)
        {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
            TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
            OCR1A = (60000 / rpm) * MS_TO_COMP(8) - 1;
            OCR1B = PWMOUT_DUTY * OCR1A;
#endif
#if defined(__AVR_ATmega2560__)
            TCCR4A |= _BV(COM4A1) | _BV(COM4B1);
            OCR4A = (60000 / rpm) * MS_TO_COMP(8) - 1;
            OCR4B = PWMOUT_DUTY * OCR4A;
#endif
        }
        else
        {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega32U4__)
            TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
#endif
#if defined(__AVR_ATmega2560__)
            TCCR4A &= ~_BV(COM4A1) & ~_BV(COM4B1);
#endif
        }
        interrupts();
    }
}

#if RX_PROTOCOL == RX_SMARTPORT
void initConfig(Config &config)
{
    smartport.deleteSensors();
    smartport.setSensorId(smartport.idToCrc(config.sensorId));
    if (config.pwmOut && config.protocol != PROTOCOL_PWM && config.protocol != PROTOCOL_CASTLE)
    {
        pwmOut = true;
        setPwmOut(config.pwmOut);
    }
    if (config.protocol == PROTOCOL_PWM)
    {
        Sensor *sensorP;
        EscPWM *esc;
        esc = new EscPWM(config.alpha.rpm);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, config.refresh.rpm, esc);
        smartport.addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        Sensor *sensorP;
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, config.alpha.rpm);
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc->begin();
        rpmPwmoutP = esc->rpmP;
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, config.refresh.rpm, esc);
        smartport.addSensor(sensorP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        Sensor *sensorP;
        EscHW4 *esc;
        ESC_SERIAL.begin(19200);
        ESC_SERIAL.setTimeout(ESCSERIAL_TIMEOUT);
        esc = new EscHW4(ESC_SERIAL, config.alpha.rpm, config.alpha.volt, config.alpha.curr, config.alpha.temp, config.protocol - PROTOCOL_HW_V4_LV);
        esc->begin();
        rpmPwmoutP = esc->rpmP;
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, ESCHW4_RPM, config.refresh.rpm, esc);
        smartport.addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, ESCHW4_CURRENT, ESCHW4_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, ESCHW4_TEMPFET, config.refresh.temp, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, ESCHW4_TEMPBEC, config.refresh.temp, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, ESCHW4_CELL_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_CASTLE)
    {
        Sensor *sensorP;
        EscCastle *esc;
        esc = new EscCastle(config.alpha.rpm, config.alpha.volt, config.alpha.curr, config.alpha.temp);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, CASTLE_RPM, config.refresh.rpm, esc);
        smartport.addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID, CASTLE_CURRENT, CASTLE_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
        sensorP = new SensorDouble(SBEC_POWER_FIRST_ID, CASTLE_BEC_CURRENT, CASTLE_BEC_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
        sensorP = new SensorDouble(ESC_POWER_FIRST_ID + 1, 0xFF, CASTLE_RIPPLE_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, CASTLE_TEMP, config.refresh.temp, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, CASTLE_TEMP_NTC, config.refresh.temp, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(VFAS_FIRST_ID, CASTLE_CELL_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
    }
    if (config.gps == true)
    {
        Sensor *sensorP;
        Bn220 *gps;
        GPS_SERIAL.begin(9600);
        GPS_SERIAL.setTimeout(BN220_TIMEOUT);
        gps = new Bn220(GPS_SERIAL);
        sensorP = new SensorLatLon(GPS_LONG_LATI_FIRST_ID, BN220_LON, BN220_LAT, config.refresh.def, gps);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(GPS_ALT_FIRST_ID, BN220_ALT, config.refresh.def, gps);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(GPS_SPEED_FIRST_ID, BN220_SPD, config.refresh.def, gps);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(GPS_COURS_FIRST_ID, BN220_COG, config.refresh.def, gps);
        smartport.addSensor(sensorP);
        sensorP = new SensorDateTime(GPS_TIME_DATE_FIRST_ID, BN220_TIME, BN220_DATE, config.refresh.def, gps);
        smartport.addSensor(sensorP);
    }
    if (config.airspeed == true)
    {
        Sensor *sensorP;
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, config.alpha.volt);
        sensorP = new Sensor(AIR_SPEED_FIRST_ID, config.refresh.volt, pressure);
        smartport.addSensor(sensorP);
    }
    if (config.voltage1 == true)
    {
        Sensor *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, config.alpha.volt);
        sensorP = new Sensor(A3_FIRST_ID, config.refresh.volt, voltage);
        smartport.addSensor(sensorP);
    }
    if (config.voltage2 == true)
    {
        Sensor *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, config.alpha.volt);
        sensorP = new Sensor(A4_FIRST_ID, config.refresh.volt, voltage);
        smartport.addSensor(sensorP);
    }
    if (config.current == true)
    {
        Sensor *sensorP;
        Voltage *voltage;
        voltage = new Voltage(PIN_CURRENT, config.alpha.curr);
        sensorP = new Sensor(CURR_FIRST_ID, config.refresh.curr, voltage);
        smartport.addSensor(sensorP);
    }
    if (config.ntc1 == true)
    {
        Sensor *sensorP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, config.alpha.temp);
        sensorP = new Sensor(T1_FIRST_ID, config.refresh.temp, ntc);
        smartport.addSensor(sensorP);
    }
    if (config.ntc2 == true)
    {
        Sensor *sensorP;
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, config.alpha.temp);
        sensorP = new Sensor(T2_FIRST_ID, config.refresh.temp, ntc);
        smartport.addSensor(sensorP);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        if (config.deviceI2C[i].type == I2C_BMP280)
        {
            Sensor *sensorP;
            Bmp280 *bmp;
            bmp = new Bmp280(config.deviceI2C[i].address, config.alpha.temp, config.alpha.def);
            bmp->begin();
            sensorP = new Sensor(T1_FIRST_ID + 2, BMP_TEMPERATURE, config.refresh.temp, bmp);
            smartport.addSensor(sensorP);
            sensorP = new Sensor(ALT_FIRST_ID + 2, BMP_ALTITUDE, config.refresh.def, bmp);
            smartport.addSensor(sensorP);
        }
    }
}

Config readConfig()
{
    Config config;
    uint32_t chk;
    EEPROM.get(0, chk);
    if (chk == 0x64616E69)
    {
        EEPROM.get(4, config);
        if (config.refresh.rpm > 16)
            config.refresh.rpm = 2;
        if (config.refresh.volt > 16)
            config.refresh.volt = 10;
        if (config.refresh.curr > 16)
            config.refresh.curr = 10;
        if (config.refresh.temp > 16)
            config.refresh.temp = 10;
        if (config.alpha.rpm > 100)
            config.alpha.rpm = 100;
        if (config.alpha.volt > 100)
            config.alpha.volt = 100;
        if (config.alpha.curr > 100)
            config.alpha.curr = 100;
        if (config.alpha.temp > 100)
            config.alpha.temp = 100;
    }
    else
    {
        writeConfig(config);
    }
#ifdef DEBUG
    DEBUG_SERIAL.println("READ CONFIG");
    DEBUG_SERIAL.print("Prot ");
    DEBUG_SERIAL.println(config.protocol);
    DEBUG_SERIAL.print("V1 ");
    DEBUG_SERIAL.println(config.voltage1);
    DEBUG_SERIAL.print("V2 ");
    DEBUG_SERIAL.println(config.voltage2);
    DEBUG_SERIAL.print("C ");
    DEBUG_SERIAL.println(config.current);
    DEBUG_SERIAL.print("NTC1 ");
    DEBUG_SERIAL.println(config.ntc1);
    DEBUG_SERIAL.print("NTC2 ");
    DEBUG_SERIAL.println(config.ntc2);
    DEBUG_SERIAL.print("PWM ");
    DEBUG_SERIAL.println(config.pwmOut);
    DEBUG_SERIAL.print("R RPM ");
    DEBUG_SERIAL.println(config.refresh.rpm);
    DEBUG_SERIAL.print("R V ");
    DEBUG_SERIAL.println(config.refresh.volt);
    DEBUG_SERIAL.print("R C ");
    DEBUG_SERIAL.println(config.refresh.curr);
    DEBUG_SERIAL.print("R T ");
    DEBUG_SERIAL.println(config.refresh.temp);
    DEBUG_SERIAL.print("a RPM ");
    DEBUG_SERIAL.println(config.alpha.rpm);
    DEBUG_SERIAL.print("a V ");
    DEBUG_SERIAL.println(config.alpha.volt);
    DEBUG_SERIAL.print("a C ");
    DEBUG_SERIAL.println(config.alpha.curr);
    DEBUG_SERIAL.print("a T ");
    DEBUG_SERIAL.println(config.alpha.temp);
    DEBUG_SERIAL.print("I2C1 ");
    DEBUG_SERIAL.println(config.deviceI2C[0].type);
    DEBUG_SERIAL.print("Addr1 ");
    DEBUG_SERIAL.println(config.deviceI2C[0].address);
    DEBUG_SERIAL.print("I2C2 ");
    DEBUG_SERIAL.println(config.deviceI2C[1].type);
    DEBUG_SERIAL.print("Addr2 ");
    DEBUG_SERIAL.println(config.deviceI2C[1].address);
#endif
    return config;
}

void writeConfig(Config &config)
{
    EEPROM.put(0, (uint32_t)0x64616E69);
    EEPROM.put(4, config);
#ifdef DEBUG
    DEBUG_SERIAL.println("WRITE CONFIG");
    DEBUG_SERIAL.print("Prot ");
    DEBUG_SERIAL.println(config.protocol);
    DEBUG_SERIAL.print("V1 ");
    DEBUG_SERIAL.println(config.voltage1);
    DEBUG_SERIAL.print("V2 ");
    DEBUG_SERIAL.println(config.voltage2);
    DEBUG_SERIAL.print("C ");
    DEBUG_SERIAL.println(config.current);
    DEBUG_SERIAL.print("NTC1 ");
    DEBUG_SERIAL.println(config.ntc1);
    DEBUG_SERIAL.print("NTC2 ");
    DEBUG_SERIAL.println(config.ntc2);
    DEBUG_SERIAL.print("PWM ");
    DEBUG_SERIAL.println(config.pwmOut);
    DEBUG_SERIAL.print("R RPM ");
    DEBUG_SERIAL.println(config.refresh.rpm);
    DEBUG_SERIAL.print("R V ");
    DEBUG_SERIAL.println(config.refresh.volt);
    DEBUG_SERIAL.print("R C ");
    DEBUG_SERIAL.println(config.refresh.curr);
    DEBUG_SERIAL.print("R T ");
    DEBUG_SERIAL.println(config.refresh.temp);
    DEBUG_SERIAL.print("a RPM ");
    DEBUG_SERIAL.println(config.alpha.rpm);
    DEBUG_SERIAL.print("a V ");
    DEBUG_SERIAL.println(config.alpha.volt);
    DEBUG_SERIAL.print("a C ");
    DEBUG_SERIAL.println(config.alpha.curr);
    DEBUG_SERIAL.print("a T ");
    DEBUG_SERIAL.println(config.alpha.temp);
    DEBUG_SERIAL.print("I2C1 ");
    DEBUG_SERIAL.println(config.deviceI2C[0].type);
    DEBUG_SERIAL.print("Addr1 ");
    DEBUG_SERIAL.println(config.deviceI2C[0].address);
    DEBUG_SERIAL.print("I2C2 ");
    DEBUG_SERIAL.println(config.deviceI2C[1].type);
    DEBUG_SERIAL.print("Addr2 ");
    DEBUG_SERIAL.println(config.deviceI2C[1].address);
#endif
}

void processPacket(uint8_t frameId, uint16_t dataId, uint32_t value)
{
    if (smartport.maintenanceMode())
    {
        // send config
        if (frameId == 0x30 && dataId == 0x5000 && value == 0)
        {
            uint32_t value = 0;
            Config config = readConfig();
            // packet 1
            value = 0xF1;
            value |= (uint32_t)VERSION_PATCH << 8;
            value |= (uint32_t)VERSION_MINOR << 16;
            value |= (uint32_t)VERSION_MAJOR << 24;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5000, value);
            // packet 2
            value = 0xF2;
            value |= config.airspeed << 8;
            value |= config.gps << 9;
            value |= config.voltage1 << 10;
            value |= config.voltage2 << 11;
            value |= config.current << 12;
            value |= config.ntc1 << 13;
            value |= config.ntc2 << 14;
            value |= config.pwmOut << 15;
            value |= (uint32_t)config.refresh.rpm << 16;
            value |= (uint32_t)config.refresh.volt << 20;
            value |= (uint32_t)config.refresh.curr << 24;
            value |= (uint32_t)config.refresh.temp << 28;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5000, value);
            // packet 3
            value = 0xF3;
            value |= (uint32_t)(200 / config.alpha.rpm - 1) << 8;
            value |= (uint32_t)(200 / config.alpha.volt - 1) << 12;
            value |= (uint32_t)(200 / config.alpha.curr - 1) << 16;
            value |= (uint32_t)(200 / config.alpha.temp - 1) << 20;
            value |= (uint32_t)config.protocol << 24;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5000, value);
            // packet 4
            value = 0xF4;
            value |= (uint32_t)config.deviceI2C[0].type << 8;
            value |= (uint32_t)config.deviceI2C[1].type << 12;
            value |= (uint32_t)config.deviceI2C[0].address << 16;
            value |= (uint32_t)config.deviceI2C[1].address << 20;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5000, value);
            return;
        }

        // receive config
        if (frameId == 0x31 && dataId == 0x5000 && (uint8_t)(value) == 0xF1)
        {
#ifdef DEBUG
            DEBUG_SERIAL.print("PACKET 1: ");
            DEBUG_SERIAL.println(value);
#endif
            Config config;
            config.airspeed = BM_AIRSPEED(value);
            config.gps = BM_GPS(value);
            config.voltage1 = BM_VOLTAGE1(value);
            config.voltage2 = BM_VOLTAGE2(value);
            config.current = BM_CURRENT(value);
            config.ntc1 = BM_NTC1(value);
            config.ntc2 = BM_NTC2(value);
            config.pwmOut = BM_PWM(value);
            config.refresh.rpm = BM_REFRESH_RPM(value);
            config.refresh.volt = BM_REFRESH_VOLT(value);
            config.refresh.curr = BM_REFRESH_CURR(value);
            config.refresh.temp = BM_REFRESH_TEMP(value);
            while (frameId != 0x31 || dataId != 0x5000 || (uint8_t)(value) != 0xF2)
            {
                smartport.update(frameId, dataId, value);
            }
#ifdef DEBUG
            DEBUG_SERIAL.print("PACKET 2: ");
            DEBUG_SERIAL.println(value);
#endif
            config.alpha.rpm = calcAlpha(BM_AVG_ELEM_RPM(value));
            config.alpha.volt = calcAlpha(BM_AVG_ELEM_VOLT(value));
            config.alpha.curr = calcAlpha(BM_AVG_ELEM_CURR(value));
            config.alpha.temp = calcAlpha(BM_AVG_ELEM_TEMP(value));
            config.protocol = BM_PROTOCOL(value);
            while (frameId != 0x31 || dataId != 0x5000 || (uint8_t)(value) != 0xF3)
            {
                smartport.update(frameId, dataId, value);
            }
#ifdef DEBUG
            DEBUG_SERIAL.print("PACKET 3: ");
            DEBUG_SERIAL.println(value);
#endif
            config.deviceI2C[0].type = BM_I2C1(value);
            config.deviceI2C[1].type = BM_I2C2(value);
            config.deviceI2C[0].address = BM_I2C1_ADDRESS(value);
            config.deviceI2C[1].address = BM_I2C2_ADDRESS(value);
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5000, 0xFF);
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            writeConfig(config);
            initConfig(config);
        }
    }
}
#endif

void setup()
{
#if defined(DEBUG) || defined(DEBUG_ESC)
    // DEBUG is on Serial
    // Baud rate depends on whats connected to Serial
    // For boards with 1 UART: If is an ESC serial it will be changed to 19600. If it is a GPS, to 9600
    // Otherwise it is at 115200
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL);
    DEBUG_SERIAL.println("\nDEBUG");
    DEBUG_SERIAL.print("V");
    DEBUG_SERIAL.print(VERSION_MAJOR);
    DEBUG_SERIAL.print(".");
    DEBUG_SERIAL.print(VERSION_MINOR);
    DEBUG_SERIAL.print(".");
    DEBUG_SERIAL.println(VERSION_PATCH);
#endif
#if RX_PROTOCOL == RX_SMARTPORT
    SMARTPORT_SRXL_SERIAL.begin(57600);
    smartport.setDataId(DATA_ID);
#endif
#if RX_PROTOCOL != RX_XBUS
    Wire.begin();
    Wire.setTimeout(WIRE_TIMEOUT);
#endif
#if RX_PROTOCOL == RX_XBUS
    xbus.begin();
#endif
#if RX_PROTOCOL == RX_SRXL
    SMARTPORT_SRXL_SERIAL.begin(115200);
    SMARTPORT_SRXL_SERIAL.setTimeout(SRXLSERIAL_TIMEOUT);
    srxl.begin(SMARTPORT_SRXL_SERIAL);
#endif
#if defined(CONFIG_LUA) && RX_PROTOCOL == RX_SMARTPORT
    Config config = readConfig();
#else
    Config config;
#endif
#ifdef DEBUG
    delay(100);
#endif
#if RX_PROTOCOL == RX_SMARTPORT
    initConfig(config);
#endif
}

void loop()
{
#if RX_PROTOCOL == RX_SMARTPORT
    uint8_t frameId;
    uint16_t dataId;
    uint32_t value;
    uint8_t result = smartport.update(frameId, dataId, value);
    if (result == RECEIVED_PACKET)
    {
        processPacket(frameId, dataId, value);
    }
    else if (result == CHANGED_SENSOR_ID)
    {
        Config config = readConfig();
        config.sensorId = smartport.crcToId(smartport.sensorId());
        writeConfig(config);
    }
#endif
    if (pwmOut)
    {
        updatePwmOut();
    }
#if RX_PROTOCOL == RX_XBUS
    xbus.update();
#endif
#if RX_PROTOCOL == RX_SRXL
    srxl.update();
    srxl.checkSerial();
#endif
#ifdef DEBUG_PLOTTER
    DEBUG_SERIAL.println(DEBUG_PLOTTER);
#endif
}