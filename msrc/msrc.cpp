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
    Serial.println("READ CONFIG");
#endif
#ifdef DEBUG2
    escSerial.println("Read cfg");
    escSerial.print("Prot ");
    escSerial.println(config.protocol);
    escSerial.print("V1 ");
    escSerial.println(config.voltage1);
    escSerial.print("V2 ");
    escSerial.println(config.voltage2);
    escSerial.print("C ");
    escSerial.println(config.current);
    escSerial.print("NTC1 ");
    escSerial.println(config.ntc1);
    escSerial.print("NTC2 ");
    escSerial.println(config.ntc2);
    escSerial.print("PWM ");
    escSerial.println(config.pwmOut);
    escSerial.print("R RPM ");
    escSerial.println(config.refresh.rpm);
    escSerial.print("R V ");
    escSerial.println(config.refresh.volt);
    escSerial.print("R C ");
    escSerial.println(config.refresh.curr);
    escSerial.print("R T ");
    escSerial.println(config.refresh.temp);
    escSerial.print("a RPM ");
    escSerial.println(config.alpha.rpm);
    escSerial.print("a V ");
    escSerial.println(config.alpha.volt);
    escSerial.print("a C ");
    escSerial.println(config.alpha.curr);
    escSerial.print("a T ");
    escSerial.println(config.alpha.temp);
    escSerial.print("I2C1 ");
    escSerial.println(config.deviceI2C[0].type);
    escSerial.print("Addr1 ");
    escSerial.println(config.deviceI2C[0].address);
    escSerial.print("I2C2 ");
    escSerial.println(config.deviceI2C[1].type);
    escSerial.print("Addr2 ");
    escSerial.println(config.deviceI2C[1].address);
#endif
    return config;
}

void writeConfig(Config &config)
{
    EEPROM.put(0, (uint32_t)0x64616E69);
    EEPROM.put(4, config);
#ifdef DEBUG
    Serial.println("WRITE CONFIG");
#endif
#ifdef DEBUG2
    escSerial.println("Write cfg");
    escSerial.print("Prot ");
    escSerial.println(config.protocol);
    escSerial.print("V1 ");
    escSerial.println(config.voltage1);
    escSerial.print("V2 ");
    escSerial.println(config.voltage2);
    escSerial.print("C ");
    escSerial.println(config.current);
    escSerial.print("NTC1 ");
    escSerial.println(config.ntc1);
    escSerial.print("NTC2 ");
    escSerial.println(config.ntc2);
    escSerial.print("PWM ");
    escSerial.println(config.pwmOut);
    escSerial.print("R RPM ");
    escSerial.println(config.refresh.rpm);
    escSerial.print("R V ");
    escSerial.println(config.refresh.volt);
    escSerial.print("R C ");
    escSerial.println(config.refresh.curr);
    escSerial.print("R T ");
    escSerial.println(config.refresh.temp);
    escSerial.print("a RPM ");
    escSerial.println(config.alpha.rpm);
    escSerial.print("a V ");
    escSerial.println(config.alpha.volt);
    escSerial.print("a C ");
    escSerial.println(config.alpha.curr);
    escSerial.print("a T ");
    escSerial.println(config.alpha.temp);
    escSerial.print("I2C1 ");
    escSerial.println(config.deviceI2C[0].type);
    escSerial.print("Addr1 ");
    escSerial.println(config.deviceI2C[0].address);
    escSerial.print("I2C2 ");
    escSerial.println(config.deviceI2C[1].type);
    escSerial.print("Addr2 ");
    escSerial.println(config.deviceI2C[1].address);
#endif
}

void setPwmOut(bool pwmOut)
{
    noInterrupts();
    if (pwmOut)
    {
        // Init pin
        pinMode(PIN_PWM_OUT_OCR, OUTPUT);
        digitalWrite(PIN_PWM_OUT_OCR, LOW);
        // Set timer1: WGM mode 15 (OCR), scaler 8 (OC1B, PB2, pin 10)
        TCCR1A = _BV(WGM11) | _BV(WGM10);
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    }
    else
    {
        TCCR1A &= ~_BV(COM1A1) & ~_BV(COM1B1);
    }
    interrupts();
}

void initConfig(Config &config)
{
    smartport.deleteSensors();
    smartport.setSensorId(smartport.idToCrc(config.sensorId));
    if (config.pwmOut && config.protocol != PROTOCOL_PWM)
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
        rpmSensor = sensorP;
        smartport.addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_HW_V3)
    {
        Sensor *sensorP;
        EscHW3Interface *esc;
        esc = new EscHW3Interface(escSerial, config.alpha.rpm);
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, config.refresh.rpm, esc);
        rpmSensor = sensorP;
        smartport.addSensor(sensorP);
    }
    if (config.protocol == PROTOCOL_HW_V4)
    {
        Sensor *sensorP;
        EscHW4Interface *esc;
        esc = new EscHW4Interface(escSerial, config.alpha.rpm, config.alpha.volt, config.alpha.curr, config.alpha.temp);
        sensorP = new Sensor(ESC_RPM_CONS_FIRST_ID, ESCHW4_RPM, config.refresh.rpm, esc);
        rpmSensor = sensorP;
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_POWER_FIRST_ID, ESCHW4_CURRENT, ESCHW4_VOLTAGE, config.refresh.volt, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID, ESCHW4_TEMPFET, config.refresh.temp, esc);
        smartport.addSensor(sensorP);
        sensorP = new Sensor(ESC_TEMPERATURE_FIRST_ID + 1, ESCHW4_TEMPBEC, config.refresh.temp, esc);
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

            sensorP = new Sensor(T1_FIRST_ID, BMP_TEMPERATURE, config.refresh.temp, bmp);
            smartport.addSensor(sensorP);
            sensorP = new Sensor(ALT_FIRST_ID, BMP_ALTITUDE, config.refresh.def, bmp);
            smartport.addSensor(sensorP);
        }
        if (config.deviceI2C[i].type == I2C_BMP280)
        {
            Sensor *sensorP;
            Bmp280Interface *bmp;
            bmp = new Bmp280Interface(config.deviceI2C[i].address, config.alpha.temp, config.alpha.def);
            bmp->begin();
            sensorP = new Sensor(T1_FIRST_ID + 1, BMP_TEMPERATURE, config.refresh.temp, bmp);
            smartport.addSensor(sensorP);
            sensorP = new Sensor(ALT_FIRST_ID + 1, BMP_ALTITUDE, config.refresh.def, bmp);
            smartport.addSensor(sensorP);
        }
        /*if (config.deviceI2C[i].type == I2C_MS5611)
            MS5611 *ms5611; 
            ms5611 = new MS5611(config.deviceI2C[i].address, config.alpha.temp, config.alpha.def);
            ms5611->begin();
            sensorP = new Sensor_ms5611Temp(T1_FIRST_ID + 2, BMP_TEMPERATURE, config.refresh.temp, bmp);
            smartport.addSensor(sensorP);
            sensorP = new Sensor_ms5611Alt(ALT_FIRST_ID + 2, BMP_ALTITUDE, config.refresh.def, bmp);
            smartport.addSensor(sensorP);
            Serial.println("MS5611");
            }
            */
    }
}

void processPacket(uint8_t frameId, uint16_t dataId, uint32_t value)
{
    if (smartport.maintenanceMode())
    {
        // send config
        if (frameId == 0x30 && dataId == 0x5000)
        {
            uint16_t timestamp = millis();
            uint32_t value = 0;
            Config config = readConfig();
            // packet 1
            value = VERSION_PATCH;
            value |= (uint32_t)VERSION_MINOR << 8;
            value |= (uint32_t)VERSION_MAJOR << 16;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5001, value);
            // packet 2
            value = config.protocol;
            value |= config.voltage1 << 2;
            value |= config.voltage2 << 3;
            value |= config.current << 4;
            value |= config.ntc1 << 5;
            value |= config.ntc2 << 6;
            value |= config.pwmOut << 7;
            value |= (uint32_t)config.refresh.rpm << 8;
            value |= (uint32_t)config.refresh.volt << 12;
            value |= (uint32_t)config.refresh.curr << 16;
            value |= (uint32_t)config.refresh.temp << 20;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5002, value);
            // packet 3
            value = (uint32_t)(200 / config.alpha.rpm - 1);
            value |= (uint32_t)(200 / config.alpha.volt - 1) << 4;
            value |= (uint32_t)(200 / config.alpha.curr - 1) << 8;
            value |= (uint32_t)(200 / config.alpha.temp - 1) << 12;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5003, value);
            // packet 4
            value = (uint32_t)config.deviceI2C[0].type;
            value |= (uint32_t)config.deviceI2C[1].type << 4;
            value |= (uint32_t)config.deviceI2C[0].address << 8;
            value |= (uint32_t)config.deviceI2C[1].address << 16;
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5004, value);
            return;
        }
        // receive config
        if (frameId == 0x30 && dataId == 0x5011)
        {
#ifdef DEBUG
            Serial.print("PACKET 1 RECEIVED: ");
            Serial.println(value);
#endif
            uint16_t timestamp = millis();
            Config config;
            config.protocol = BM_PROTOCOL(value);
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
            while (frameId != 0x30 || dataId != 0x5012)
            {
                smartport.update(frameId, dataId, value);
            }
#ifdef DEBUG
            Serial.print("PACKET 2 RECEIVED: ");
            Serial.println(value);
#endif
            config.alpha.rpm = calcAlpha(BM_AVG_ELEM_RPM(value));
            config.alpha.volt = calcAlpha(BM_AVG_ELEM_VOLT(value));
            config.alpha.curr = calcAlpha(BM_AVG_ELEM_CURR(value));
            config.alpha.temp = calcAlpha(BM_AVG_ELEM_TEMP(value));
            while (frameId != 0x30 || dataId != 0x5013)
            {
                smartport.update(frameId, dataId, value);
            }
#ifdef DEBUG
            Serial.print("PACKET 3 RECEIVED: ");
            Serial.println(value);
#endif
            config.deviceI2C[0].type = BM_I2C1(value);
            config.deviceI2C[1].type = BM_I2C2(value);
            config.deviceI2C[0].address = BM_I2C1_ADDRESS(value);
            config.deviceI2C[1].address = BM_I2C2_ADDRESS(value);
            while (!smartport.sendPacketReady())
            {
                smartport.update();
            }
            smartport.addPacket(0x32, 0x5020, 0);
            writeConfig(config);
            initConfig(config);
        }   
    }
}

void updatePwmOut()
{
    static float rpm = 0;
    if (rpmSensor->valueL() != rpm)
    {
        rpm = rpmSensor->valueL();
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

void setup()
{
    escSerial.begin(19200);
    escSerial.setTimeout(ESCSERIAL_TIMEOUT);
    smartportSerial.begin(57600);
    Wire.begin();
    Wire.setTimeout(WIRE_TIMEOUT);
    smartport.setDataId(DATA_ID);
    TIMSK1 = 0;
#ifdef DEBUG
    escSerial.println("\nDEBUG");
    escSerial.print("V");
    escSerial.print(VERSION_MAJOR);
    escSerial.print(".");
    escSerial.print(VERSION_MINOR);
    escSerial.print(".");
    escSerial.println(VERSION_PATCH);
#endif
#ifdef CONFIG_LUA
    Config config = readConfig();
#else
    Config config;
#endif
    initConfig(config);
}

void loop()
{
    uint8_t frameId;
    uint16_t dataId;
    uint32_t value;
    if (smartport.update(frameId, dataId, value) == RECEIVED_PACKET)
    {
        processPacket(frameId, dataId, value);
    }
    else if (smartport.update() == CHANGED_SENSOR_ID)
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
    Serial.println(DEBUG_PLOTTER);
#endif
}