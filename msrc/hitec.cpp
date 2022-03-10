#include "hitec.h"

volatile bool Hitec::isEnabledFrame[11] = {0};
//volatile bool Hitec::isEnabledFrame[11] = {1,1,1,1,1,1,1,1,1,1,1};
bool Hitec::isEmpty = true;
float *Hitec::frame_0x11_P[1] = {NULL};
float *Hitec::frame_0x12_P[2] = {NULL};
float *Hitec::frame_0x13_P[2] = {NULL};
float *Hitec::frame_0x14_P[3] = {NULL};
float *Hitec::frame_0x15_P[3] = {NULL};
float *Hitec::frame_0x16_P[2] = {NULL};
float *Hitec::frame_0x17_P[4] = {NULL};
float *Hitec::frame_0x18_P[2] = {NULL};
float *Hitec::frame_0x19_P[4] = {NULL};
float *Hitec::frame_0x1A_P[1] = {NULL};
float *Hitec::frame_0x1B_P[2] = {NULL};

Hitec::Hitec()
{
}

void Hitec::begin()
{
    Wire.begin(HITEC_I2C_ADDRESS);
    Wire.onRequest(i2c_request_handler);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
    if (deviceElementP)
        isEmpty = false;
}

void Hitec::i2c_request_handler()
{
    if (!isEmpty)
    {
        static uint8_t frame = 0;
        int32_t valueS32;
        uint16_t valueU16;
        uint16_t valueS16;
        uint8_t valueU8;
        uint8_t buffer[7] = {0};
        do
        {
            frame++;
            frame %= 11;
        } while (!isEnabledFrame[frame]);
        buffer[0] = frame + 0x11;
        buffer[6] = frame + 0x11;
        switch (frame)
        {
        case HITEC_FRAME_0X11:
            buffer[1] = 0xAF;
            buffer[3] = 0x2D;
            if (frame_0x11_P[HITEC_FRAME_0X11_RX_BATT])
            {
                valueU16 = *frame_0x11_P[HITEC_FRAME_0X11_RX_BATT] * 28;
                buffer[4] = valueU16 >> 8;
                buffer[5] = valueU16;
            }
            break;
        case HITEC_FRAME_0X12:
            if (frame_0x12_P[HITEC_FRAME_0X12_GPS_LAT])
            {
                float degF = *frame_0x12_P[HITEC_FRAME_0X12_GPS_LAT] / 60;
                int8_t deg = degF;
                int8_t min = (degF - deg) * 60;
                float sec = ((degF - deg) * 60 - min) * 60;
                int16_t sec_x_100 = sec * 100;
                int16_t deg_min = deg * 100 + min;
                buffer[1] = sec_x_100 >> 8;
                buffer[2] = sec_x_100;
                buffer[3] = deg_min >> 8;
                buffer[4] = deg_min; 
            }
            if (frame_0x12_P[HITEC_FRAME_0X12_TIME])
            {
                valueU8 = *frame_0x12_P[HITEC_FRAME_0X12_TIME];
            }
            break;
        case HITEC_FRAME_0X13:
            if (frame_0x13_P[HITEC_FRAME_0X13_GPS_LON])
            {
                float degF = *frame_0x13_P[HITEC_FRAME_0X13_GPS_LON] / 60;
                int8_t deg = degF;
                int8_t min = (degF - deg) * 60;
                float sec = ((degF - deg) * 60 - min) * 60;
                int16_t sec_x_100 = sec * 100;
                int16_t deg_min = deg * 100 + min;
                buffer[1] = sec_x_100 >> 8;
                buffer[2] = sec_x_100;
                buffer[3] = deg_min >> 8;
                buffer[4] = deg_min;
            }
            if (frame_0x13_P[HITEC_FRAME_0X13_TEMP2])
            {
                valueU8 = round(*frame_0x13_P[HITEC_FRAME_0X13_TEMP2] + 40);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X14:
            if (frame_0x14_P[HITEC_FRAME_0X14_GPS_SPD])
            {
                valueU16 = round(*frame_0x14_P[HITEC_FRAME_0X14_GPS_SPD] * 1.852);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (frame_0x14_P[HITEC_FRAME_0X14_GPS_ALT])
            {
                valueS16 = round(*frame_0x14_P[HITEC_FRAME_0X14_GPS_ALT]);
                buffer[3] = valueS16 >> 8;
                buffer[4] = valueS16;
            }
            if (frame_0x14_P[HITEC_FRAME_0X14_TEMP1])
            {
                valueU8 = round(*frame_0x14_P[HITEC_FRAME_0X14_TEMP1] + 40);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X15:
            if (frame_0x15_P[HITEC_FRAME_0X15_RPM1])
            {
                valueU16 = round(*frame_0x15_P[HITEC_FRAME_0X15_RPM1]);
                buffer[2] = valueU16;
                buffer[3] = valueU16 >> 8;
            }
            if (frame_0x15_P[HITEC_FRAME_0X15_RPM2])
            {
                valueU16 = round(*frame_0x15_P[HITEC_FRAME_0X15_RPM2]);
                buffer[4] = valueU16;
                buffer[5] = valueU16 >> 8;
            }
            break;
        case HITEC_FRAME_0X16:
            if (frame_0x16_P[HITEC_FRAME_0X16_DATE])
            {
                valueS32 = *frame_0x16_P[HITEC_FRAME_0X16_DATE];
                buffer[3] = valueS32 / 10000;                                 // year
                buffer[2] = (valueS32 - buffer[3] * 10000UL) / 100;           // month
                buffer[1] = valueS32 - buffer[3] * 10000UL - buffer[2] * 100; // day
            }
            if (frame_0x16_P[HITEC_FRAME_0X16_TIME])
            {
                valueS32 = *frame_0x16_P[HITEC_FRAME_0X16_TIME];
                buffer[4] = valueS32 / 10000;                       // hour
                buffer[5] = (valueS32 - buffer[4] * 10000UL) / 100; // minute
            }
            break;
        case HITEC_FRAME_0X17:
            if (frame_0x17_P[HITEC_FRAME_0X17_COG])
            {
                valueU16 = round(*frame_0x17_P[HITEC_FRAME_0X17_COG]);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (frame_0x17_P[HITEC_FRAME_0X17_SATS])
            {
                valueU8 = *frame_0x17_P[HITEC_FRAME_0X17_SATS];
                buffer[3] = valueU8;
            }
            if (frame_0x17_P[HITEC_FRAME_0X17_TEMP3])
            {
                valueU8 = round(*frame_0x17_P[HITEC_FRAME_0X17_TEMP3] + 40);
                buffer[4] = valueU8;
            }
            if (frame_0x17_P[HITEC_FRAME_0X17_TEMP4])
            {
                valueU8 = round(*frame_0x17_P[HITEC_FRAME_0X17_TEMP4] + 40);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X18:
            if (frame_0x18_P[HITEC_FRAME_0X18_VOLT])
            {
                valueU16 = round((*frame_0x18_P[HITEC_FRAME_0X18_VOLT] - 0.2) * 10);
                buffer[1] = valueU16;
                buffer[2] = valueU16 >> 8;
            }
            if (frame_0x18_P[HITEC_FRAME_0X18_AMP])
            {
                /* value for stock transmitter (tbc) */
                //valueU16 = (*frame_0x18_P[HITEC_FRAME_0X18_AMP] + 114.875) * 1.441;

                /* value for opentx transmitter  */
                valueU16 = round(*frame_0x18_P[HITEC_FRAME_0X18_AMP]);

                buffer[3] = valueU16;
                buffer[4] = valueU16 >> 8;
            }
            break;
        case HITEC_FRAME_0X19:
            if (frame_0x19_P[HITEC_FRAME_0X19_AMP1])
            {
                valueU8 = round(*frame_0x19_P[HITEC_FRAME_0X19_AMP1] * 10);
                buffer[5] = valueU8;
            }
            if (frame_0x19_P[HITEC_FRAME_0X19_AMP2])
            {
                valueU8 = round(*frame_0x19_P[HITEC_FRAME_0X19_AMP2] * 10);
                buffer[5] = valueU8;
            }
            if (frame_0x19_P[HITEC_FRAME_0X19_AMP3])
            {
                valueU8 = round(*frame_0x19_P[HITEC_FRAME_0X19_AMP3] * 10);
                buffer[5] = valueU8;
            }
            if (frame_0x19_P[HITEC_FRAME_0X19_AMP4])
            {
                valueU8 = round(*frame_0x19_P[HITEC_FRAME_0X19_AMP4] * 10);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X1A:
            if (frame_0x1A_P[HITEC_FRAME_0X1A_ASPD])
            {
                valueU16 = round(*frame_0x1A_P[HITEC_FRAME_0X1A_ASPD]);
                buffer[3] = valueU16 >> 8;
                buffer[4] = valueU16;
            }
            break;
        case HITEC_FRAME_0X1B:
            if (frame_0x1B_P[HITEC_FRAME_0X1B_ALTU])
            {
                valueU16 = round(*frame_0x1B_P[HITEC_FRAME_0X1B_ALTU]);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (frame_0x1B_P[HITEC_FRAME_0X1B_ALTF])
            {
                valueU16 = round(*frame_0x1B_P[HITEC_FRAME_0X1B_ALTF]);
                buffer[3] = valueU16 >> 8;
                buffer[4] = valueU16;
            }
            break;
        }
        digitalWrite(LED_BUILTIN, HIGH);
        Wire.write(buffer, 7);
        digitalWrite(LED_BUILTIN, LOW);
#ifdef DEBUG
        for (int i = 0; i < 7; i++)
        {
            DEBUG_PRINT_HEX(buffer[i]);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINTLN();
#endif
    }
}

void Hitec::addDevice(AbstractDevice *deviceP)
{
    DeviceElement *newDeviceElementP;
    newDeviceElementP = new DeviceElement;
    newDeviceElementP->deviceP = deviceP;
    if (deviceElementP == NULL)
    {
        deviceElementP = newDeviceElementP;
        newDeviceElementP->nextP = newDeviceElementP;
    }
    else
    {
        newDeviceElementP->nextP = deviceElementP->nextP;
        deviceElementP->nextP = newDeviceElementP;
        deviceElementP = newDeviceElementP;
    }
}

void Hitec::update()
{
    if (deviceElementP)
    {
        deviceElementP->deviceP->update();
        deviceElementP = deviceElementP->nextP;
    }
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)(millis() - ts) > 500)
    {
        ts = millis();
        i2c_request_handler();
    }
#endif
}

void Hitec::setConfig(Config &config)
{
    if (ESC_PROTOCOL == PROTOCOL_PWM)
    {
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        addDevice(esc);
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V3)
    {
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        addDevice(esc);
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        addDevice(esc);
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = esc->voltageP();
        frame_0x18_P[HITEC_FRAME_0X18_AMP] = esc->currentP();
        frame_0x14_P[HITEC_FRAME_0X14_TEMP1] = esc->tempFetP();
        frame_0x13_P[HITEC_FRAME_0X13_TEMP2] = esc->tempBecP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
        isEnabledFrame[HITEC_FRAME_0X18] = true;
        isEnabledFrame[HITEC_FRAME_0X14] = true;
        isEnabledFrame[HITEC_FRAME_0X13] = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        addDevice(esc);
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = esc->voltageP();
        frame_0x18_P[HITEC_FRAME_0X18_AMP] = esc->currentP();
        frame_0x14_P[HITEC_FRAME_0X14_TEMP1] = esc->temperatureP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
        isEnabledFrame[HITEC_FRAME_0X18] = true;
        isEnabledFrame[HITEC_FRAME_0X14] = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        addDevice(esc);
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = esc->voltageP();
        frame_0x18_P[HITEC_FRAME_0X18_AMP] = esc->currentP();
        frame_0x14_P[HITEC_FRAME_0X14_TEMP1] = esc->tempFetP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
        isEnabledFrame[HITEC_FRAME_0X18] = true;
        isEnabledFrame[HITEC_FRAME_0X14] = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        addDevice(esc);
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = esc->voltageP();
        frame_0x18_P[HITEC_FRAME_0X18_AMP] = esc->currentP();
        frame_0x14_P[HITEC_FRAME_0X14_TEMP1] = esc->tempP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
        isEnabledFrame[HITEC_FRAME_0X18] = true;
        isEnabledFrame[HITEC_FRAME_0X14] = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        addDevice(esc);
        frame_0x15_P[HITEC_FRAME_0X15_RPM1] = esc->rpmP();
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = esc->voltageP();
        frame_0x18_P[HITEC_FRAME_0X18_AMP] = esc->currentP();
        frame_0x14_P[HITEC_FRAME_0X14_TEMP1] = esc->tempP();
        isEnabledFrame[HITEC_FRAME_0X15] = true;
        isEnabledFrame[HITEC_FRAME_0X18] = true;
        isEnabledFrame[HITEC_FRAME_0X14] = true;
    }
    if (config.gps == true)
    {
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        addDevice(gps);
        frame_0x17_P[HITEC_FRAME_0X17_SATS] = gps->satP();
        frame_0x12_P[HITEC_FRAME_0X12_GPS_LAT] = gps->latP();
        frame_0x13_P[HITEC_FRAME_0X13_GPS_LON] = gps->lonP();
        frame_0x14_P[HITEC_FRAME_0X14_GPS_ALT] = gps->altP();
        frame_0x14_P[HITEC_FRAME_0X14_GPS_SPD] = gps->spdP();
        frame_0x17_P[HITEC_FRAME_0X17_COG] = gps->cogP();
        frame_0x16_P[HITEC_FRAME_0X16_DATE] = gps->dateP();
        frame_0x16_P[HITEC_FRAME_0X16_TIME] = gps->timeP();
        isEnabledFrame[HITEC_FRAME_0X17] = true;
        isEnabledFrame[HITEC_FRAME_0X12] = true;
        isEnabledFrame[HITEC_FRAME_0X13] = true;
        isEnabledFrame[HITEC_FRAME_0X14] = true;
        isEnabledFrame[HITEC_FRAME_0X16] = true;
    }
    if (config.airspeed == true)
    {
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        addDevice(pressure);
        frame_0x1A_P[HITEC_FRAME_0X1A_ASPD] = pressure->valueP();
        isEnabledFrame[HITEC_FRAME_0X1A] = true;
    }
    if (config.voltage1 == true)
    {
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        addDevice(voltage);
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = voltage->valueP();
        isEnabledFrame[HITEC_FRAME_0X18] = true;
    }
    if (config.voltage2 == true)
    {
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        addDevice(voltage);
        frame_0x18_P[HITEC_FRAME_0X18_VOLT] = voltage->valueP();
        isEnabledFrame[HITEC_FRAME_0X18] = true;
    }
    if (config.current == true)
    {
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        addDevice(current);
        frame_0x18_P[HITEC_FRAME_0X18_AMP] = current->valueP();
        isEnabledFrame[HITEC_FRAME_0X18] = true;
    }
    if (config.ntc1 == true)
    {
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        addDevice(ntc);
        frame_0x17_P[HITEC_FRAME_0X17_TEMP3] = ntc->valueP();
        isEnabledFrame[HITEC_FRAME_0X17] = true;
    }
    if (config.ntc2 == true)
    {
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        addDevice(ntc);
        frame_0x17_P[HITEC_FRAME_0X17_TEMP4] = ntc->valueP();
        isEnabledFrame[HITEC_FRAME_0X17] = true;
    }
#if (defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) && defined(I2C_T3_TEENSY)
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        addDevice(bmp);
        frame_0x1B_P[HITEC_FRAME_0X14_GPS_ALT] = bmp->altitudeP();
        isEnabledFrame[HITEC_FRAME_0X14] = true;
    }
    if (config.deviceI2C1Type == I2C_MS5611)
    {
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        addDevice(bmp);
        frame_0x1B_P[HITEC_FRAME_0X14_GPS_ALT] = bmp->altitudeP();
        isEnabledFrame[HITEC_FRAME_0X14] = true;
    }
#endif
}