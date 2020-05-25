#include "msrc.h"

uint8_t calcAlpha(uint8_t elements)
{
    return round((2.0F / (elements + 1)) * 100);
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
    debugSerial.println("READ CONFIG");
#endif
#ifdef DEBUG2
    debugSerial.println("Read cfg");
    debugSerial.print("Prot ");
    debugSerial.println(config.protocol);
    debugSerial.print("V1 ");
    debugSerial.println(config.voltage1);
    debugSerial.print("V2 ");
    debugSerial.println(config.voltage2);
    debugSerial.print("C ");
    debugSerial.println(config.current);
    debugSerial.print("NTC1 ");
    debugSerial.println(config.ntc1);
    debugSerial.print("NTC2 ");
    debugSerial.println(config.ntc2);
    debugSerial.print("PWM ");
    debugSerial.println(config.pwmOut);
    debugSerial.print("R RPM ");
    debugSerial.println(config.refresh.rpm);
    debugSerial.print("R V ");
    debugSerial.println(config.refresh.volt);
    debugSerial.print("R C ");
    debugSerial.println(config.refresh.curr);
    debugSerial.print("R T ");
    debugSerial.println(config.refresh.temp);
    debugSerial.print("a RPM ");
    debugSerial.println(config.alpha.rpm);
    debugSerial.print("a V ");
    debugSerial.println(config.alpha.volt);
    debugSerial.print("a C ");
    debugSerial.println(config.alpha.curr);
    debugSerial.print("a T ");
    debugSerial.println(config.alpha.temp);
    debugSerial.print("I2C1 ");
    debugSerial.println(config.deviceI2C[0].type);
    debugSerial.print("Addr1 ");
    debugSerial.println(config.deviceI2C[0].address);
    debugSerial.print("I2C2 ");
    debugSerial.println(config.deviceI2C[1].type);
    debugSerial.print("Addr2 ");
    debugSerial.println(config.deviceI2C[1].address);
#endif
    return config;
}

void writeConfig(Config &config)
{
    EEPROM.put(0, (uint32_t)0x64616E69);
    EEPROM.put(4, config);
#ifdef DEBUG
    debugSerial.println("WRITE CONFIG");
#endif
#ifdef DEBUG2
    debugSerial.println("Write cfg");
    debugSerial.print("Prot ");
    debugSerial.println(config.protocol);
    debugSerial.print("V1 ");
    debugSerial.println(config.voltage1);
    debugSerial.print("V2 ");
    debugSerial.println(config.voltage2);
    debugSerial.print("C ");
    debugSerial.println(config.current);
    debugSerial.print("NTC1 ");
    debugSerial.println(config.ntc1);
    debugSerial.print("NTC2 ");
    debugSerial.println(config.ntc2);
    debugSerial.print("PWM ");
    debugSerial.println(config.pwmOut);
    debugSerial.print("R RPM ");
    debugSerial.println(config.refresh.rpm);
    debugSerial.print("R V ");
    debugSerial.println(config.refresh.volt);
    debugSerial.print("R C ");
    debugSerial.println(config.refresh.curr);
    debugSerial.print("R T ");
    debugSerial.println(config.refresh.temp);
    debugSerial.print("a RPM ");
    debugSerial.println(config.alpha.rpm);
    debugSerial.print("a V ");
    debugSerial.println(config.alpha.volt);
    debugSerial.print("a C ");
    debugSerial.println(config.alpha.curr);
    debugSerial.print("a T ");
    debugSerial.println(config.alpha.temp);
    debugSerial.print("I2C1 ");
    debugSerial.println(config.deviceI2C[0].type);
    debugSerial.print("Addr1 ");
    debugSerial.println(config.deviceI2C[0].address);
    debugSerial.print("I2C2 ");
    debugSerial.println(config.deviceI2C[1].type);
    debugSerial.print("Addr2 ");
    debugSerial.println(config.deviceI2C[1].address);
#endif
}

void setPwmOut(bool pwmOut)
{
    noInterrupts();
    if (pwmOut)
    {
        // TIMER1 setup: mode 15 (OCR), scaler 8 (OC1B, PB2, pin 10)
        DDRB |= _BV(DDB2);
        TCCR1A = _BV(WGM11) | _BV(WGM10);
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    }
    else
    {
        TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
    }
    interrupts();
}

void updatePwmOut()
{
    static float rpm = 0;
    if (rpmSensorP->valueL() != rpm)
    {
        rpm = rpmSensorP->valueL();
        noInterrupts();
        if (rpm >= 2000)
        {
            TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
            OCR1A = (7.5 * (uint32_t)F_CPU / rpm) - 1;
            OCR1B = DUTY * OCR1A;
        }
        else
        {
            TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
        }
        interrupts();
    }
}

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
        EscPWMInterface *esc;
        esc = new EscPWMInterface(config.alpha.rpm);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, config.refresh.rpm, esc);
        rpmSensorP = sensorP;
        smartport.addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        Sensor *sensorP;
        EscHW3Interface *esc;
        esc = new EscHW3Interface(escSerial, config.alpha.rpm);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, config.refresh.rpm, esc);
        rpmSensorP = sensorP;
        smartport.addSensor(sensorP);
    }
    if (config.protocol >= PROTOCOL_HW_V4_LV && config.protocol <= PROTOCOL_HW_V5_HV)
    {
        Sensor *sensorP;
        EscHW4Interface *esc;
        esc = new EscHW4Interface(escSerial, config.alpha.rpm, config.alpha.volt, config.alpha.curr, config.alpha.temp, config.protocol - PROTOCOL_HW_V4_LV);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, ESCHW4_RPM, config.refresh.rpm, esc);
        rpmSensorP = sensorP;
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
        EscCastleInterface *esc;
        esc = new EscCastleInterface(config.alpha.rpm, config.alpha.volt, config.alpha.curr, config.alpha.temp);
        esc->begin();
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, CASTLE_RPM, config.refresh.rpm, esc);
        rpmSensorP = sensorP;
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
        Bn220Interface *gps;
        gps = new Bn220Interface(gpsSerial);
        gps->begin();
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
    if (config.voltage1 == true)
    {
        Sensor *sensorP;
        VoltageInterface *voltage;
        voltage = new VoltageInterface(PIN_VOLTAGE1, config.alpha.volt);
        sensorP = new Sensor(A3_FIRST_ID, config.refresh.volt, voltage);
        smartport.addSensor(sensorP);
    }
    if (config.voltage2 == true)
    {
        Sensor *sensorP;
        VoltageInterface *voltage;
        voltage = new VoltageInterface(PIN_VOLTAGE2, config.alpha.volt);
        sensorP = new Sensor(A4_FIRST_ID, config.refresh.volt, voltage);
        smartport.addSensor(sensorP);
    }
    if (config.current == true)
    {
        Sensor *sensorP;
        VoltageInterface *voltage;
        voltage = new VoltageInterface(PIN_CURRENT, config.alpha.curr);
        sensorP = new Sensor(CURR_FIRST_ID, config.refresh.curr, voltage);
        smartport.addSensor(sensorP);
    }
    if (config.ntc1 == true)
    {
        Sensor *sensorP;
        NtcInterface *ntc;
        ntc = new NtcInterface(PIN_NTC1, config.alpha.temp);
        sensorP = new Sensor(T1_FIRST_ID, config.refresh.temp, ntc);
        smartport.addSensor(sensorP);
    }
    if (config.ntc2 == true)
    {
        Sensor *sensorP;
        NtcInterface *ntc;
        ntc = new NtcInterface(PIN_NTC2, config.alpha.temp);
        sensorP = new Sensor(T2_FIRST_ID, config.refresh.temp, ntc);
        smartport.addSensor(sensorP);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        if (config.deviceI2C[i].type == I2C_BMP180)
        {
            Sensor *sensorP;
            Bmp180Interface *bmp;
            bmp = new Bmp180Interface(config.deviceI2C[i].address, config.alpha.temp, config.alpha.def);
            bmp->begin();

            sensorP = new Sensor(T1_FIRST_ID + 1, BMP_TEMPERATURE, config.refresh.temp, bmp);
            smartport.addSensor(sensorP);
            sensorP = new Sensor(ALT_FIRST_ID + 1, BMP_ALTITUDE, config.refresh.def, bmp);
            smartport.addSensor(sensorP);
        }
        if (config.deviceI2C[i].type == I2C_BMP280)
        {
            Sensor *sensorP;
            Bmp280Interface *bmp;
            bmp = new Bmp280Interface(config.deviceI2C[i].address, config.alpha.temp, config.alpha.def);
            bmp->begin();
            sensorP = new Sensor(T1_FIRST_ID + 2, BMP_TEMPERATURE, config.refresh.temp, bmp);
            smartport.addSensor(sensorP);
            sensorP = new Sensor(ALT_FIRST_ID + 2, BMP_ALTITUDE, config.refresh.def, bmp);
            smartport.addSensor(sensorP);
        }
    }
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
            //value |= 1BIT_SPARE << 8;
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
            debugSerial.print("PACKET 1: ");
            debugSerial.println(value);
#endif
            Config config;
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
            debugSerial.print("PACKET 2: ");
            debugSerial.println(value);
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
            debugSerial.print("PACKET 3: ");
            debugSerial.println(value);
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

void setup()
{
#ifdef DEBUG
    debugSerial.begin(9600);
    debugSerial.println("\nDEBUG");
    debugSerial.print("V");
    debugSerial.print(VERSION_MAJOR);
    debugSerial.print(".");
    debugSerial.print(VERSION_MINOR);
    debugSerial.print(".");
    debugSerial.println(VERSION_PATCH);
#endif
    smartportSerial.begin(57600);
    Wire.begin();
    Wire.setTimeout(WIRE_TIMEOUT);
    smartport.setDataId(DATA_ID);
    TIMSK1 = 0;

#ifdef CONFIG_LUA
    Config config = readConfig();
#else
    Config config;
#endif
#ifdef DEBUG
    delay(100);
#endif
    initConfig(config);
}

void loop()
{
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
    if (pwmOut)
    {
        updatePwmOut();
    }
#ifdef DEBUG_PLOTTER
    debugSerial.println(DEBUG_PLOTTER);
#endif
}