#include "bn220.h"

Bn220Interface::Bn220Interface(HardwareSerial &serial) : serial_(serial) {}

void Bn220Interface::begin()
{
    serial_.begin(9600);
    serial_.setTimeout(BN220_TIMEOUT);
}

void Bn220Interface::update()
{
    if (serial_.available())
    {
        char c = serial_.read();
        switch (c)
        {
        case '$':
            contIndex_ = 0;
            nmeaCmd_ = BN220_UNK;
            break;
        case '\r':
        case ',':
            if (contIndex_ == 0)
            {
                if (memcmp(buffer_ + 2, "GGA", 3) == 0)
                {
                    nmeaCmd_ = BN220_GGA;
                }
                else if (memcmp(buffer_ + 2, "GLL", 3) == 0)
                {
                    nmeaCmd_ = BN220_GLL;
                }
                else if (memcmp(buffer_ + 2, "RMC", 3) == 0)
                {
                    nmeaCmd_ = BN220_RMC;
                }
                else if (memcmp(buffer_ + 2, "VTG", 3) == 0)
                {
                    nmeaCmd_ = BN220_VTG;
                }
#ifdef DEBUG_GPS
                if (nmeaCmd_)
                {
                    Serial.print("\n");
                    Serial.print(nmeaCmd_);
                    Serial.print(" ");
                    Serial.print(buffer_ + 2);
                    Serial.print(": ");
                }
#endif
            }
            else
            {
                if (nmeaCmd_ > 0)
                {
                    uint8_t i = 0;
                    while (nmeaData[nmeaCmd_][i][0] != 0)
                    {
                        if (nmeaData[nmeaCmd_][i][0] == contIndex_)
                        {
                            uint8_t type = nmeaData[nmeaCmd_][i][1];
                            parser(type, buffer_);
                        }
                        i++;
                    }
                }
            }
            contIndex_++;
            contBuff_ = 0;
            buffer_[0] = 0;
            break;
        case '\n':
            break;
        default:
            if (contBuff_ < 19)
            {
                buffer_[contBuff_] = c;
                contBuff_++;
                buffer_[contBuff_] = 0;
            }
        }
    }
}

void Bn220Interface::parser(uint8_t type, char *data)
{
    if (strlen(data))
    {
        if (type == BN220_TIME)
        {
            value_[BN220_TIME] = atol(data);
        }
        else if (type == BN220_LAT)
        {
            char degLat[3] = {0};
            float minLat;
            strncpy(degLat, data, 2);
            minLat = atof(data + 2);
            value_[BN220_LAT] = latDir_ * (atof(degLat) * 60 + minLat);
        }
        else if (type == BN220_LON)
        {
            char degLon[4] = {0};
            float minLon;
            strncpy(degLon, data, 3);
            minLon = atof(data + 3);
            value_[BN220_LON] = lonDir_ * (atof(degLon) * 60 + minLon);
        }
        else if (type == BN220_ALT)
        {
            value_[BN220_ALT] = atof(data);
        }
        else if (type == BN220_SPD)
        {
            value_[BN220_SPD] = atof(data);
        }
        else if (type == BN220_COG)
        {
            value_[BN220_COG] = atof(data);
        }
        else if (type == BN220_DATE)
        {
            value_[BN220_DATE] = atol(data);
        }
        else if (type == BN220_KPH)
        {
            value_[BN220_KPH] = atof(data);
        }
        else if (type == BN220_SAT)
        {
            value_[BN220_SAT] = atoi(data);
        }
        else if (type == BN220_LAT_SIGN)
        {
            (data[0] == 'N') ? latDir_ = 1 : latDir_ = -1;
        }
        else if (type == BN220_LON_SIGN)
        {
            (data[0] == 'E') ? lonDir_ = 1 : latDir_ = -1;
        }

#ifdef DEBUG_GPS
        Serial.print(contIndex_);
        Serial.print("(");
        Serial.print(type);
        Serial.print(",");
        Serial.print(data);
        Serial.print(",");
        Serial.print(value_[type]);
        Serial.print(") ");
#endif
    }
}

float Bn220Interface::read(uint8_t index)
{
    if (index < 9)
    {
        update();
#ifdef SIM_SENSORS
        value_[BN220_LAT] = -630;    // 10º30" +E, -W
        value_[BN220_LON] = -1250;   // 20º50" +N, -S
        value_[BN220_ALT] = 1283;    // m
        value_[BN220_SPD] = 158;     // kts
        value_[BN220_COG] = 123.32;  // º 
        value_[BN220_KPH] = 132;     //
        value_[BN220_SAT] = 10;      //
        value_[BN220_DATE] = 10101;  // yymmdd
        value_[BN220_TIME] = 20202;  // hhmmss
#endif
        return value_[index];
    }
    return 0;
}