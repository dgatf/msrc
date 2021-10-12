#include "bn220.h"

Bn220::Bn220(AbstractSerial &serial, uint32_t baud) : serial_(serial), baud_(baud)
{
}

void Bn220::begin()
{
    serial_.begin(baud_, SERIAL_8N1);
}

void Bn220::update()
{
#if !defined(DISABLE_GPS)
    while (serial_.available())
    {
        static uint8_t contIndex = 0, contBuff = 0;
        char data = serial_.read();
        switch (data)
        {
        case '$':
            contIndex = 0;
            contBuff = 0;
            nmeaCmd_ = BN220_UNK;
            break;
        case '\r':
        case ',':
            if (contIndex == 0)
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
                    DEBUG_PRINT("\n");
                    DEBUG_PRINT(nmeaCmd_);
                    DEBUG_PRINT(" ");
                    DEBUG_PRINT(buffer_ + 2);
                    DEBUG_PRINT(": ");
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
                        if (nmeaData[nmeaCmd_][i][0] == contIndex)
                        {
                            uint8_t type = nmeaData[nmeaCmd_][i][1];
                            parser(type, buffer_);
                        }
                        i++;
                    }
                }
            }
            contIndex++;
            contBuff = 0;
            buffer_[0] = 0;
            break;
        case '\n':
            break;
        default:
            if (contBuff < 19)
            {
                buffer_[contBuff] = data;
                contBuff++;
                buffer_[contBuff] = 0;
            }
        }
    }
#ifdef SIM_SENSORS
    lat_ = -630;   // 10º30" +N, -S
    lon_ = -1250;  // 20º50" +E, -W
    alt_ = 1283;   // m
    spd_ = 158;    // kts
    cog_ = 123.32; // º
    kph_ = 132;    //
    sat_ = 10;     //
    date_ = 10101; // yymmdd
    time_ = 20202; // hhmmss
#endif
}

void Bn220::parser(uint8_t type, char *data)
{
    if (strlen(data))
    {
        if (type == BN220_TIME)
        {
            time_ = atol(data);
        }
        else if (type == BN220_LAT)
        {
            char degLat[3] = {0};
            float minLat;
            strncpy(degLat, data, 2);
            minLat = atof(data + 2);
            lat_ = latDir_ * (atof(degLat) * 60 + minLat);
        }
        else if (type == BN220_LON)
        {
            char degLon[4] = {0};
            float minLon;
            strncpy(degLon, data, 3);
            minLon = atof(data + 3);
            lon_ = lonDir_ * (atof(degLon) * 60 + minLon);
        }
        else if (type == BN220_ALT)
        {
            alt_ = atof(data);
            if (alt_ < 0)
                alt_ = 0;
        }
        else if (type == BN220_SPD)
        {
            spd_ = atof(data);
        }
        else if (type == BN220_COG)
        {
            cog_ = atof(data);
        }
        else if (type == BN220_DATE)
        {
            date_ = atol(data);
        }
        else if (type == BN220_KPH)
        {
            kph_ = atof(data);
        }
        else if (type == BN220_SAT)
        {
            sat_ = atoi(data);
        }
        else if (type == BN220_LAT_SIGN)
        {
            (data[0] == 'N') ? latDir_ = 1 : latDir_ = -1;
        }
        else if (type == BN220_LON_SIGN)
        {
            (data[0] == 'E') ? lonDir_ = 1 : lonDir_ = -1;
        }
#ifdef DEBUG_GPS
        DEBUG_PRINT("(");
        DEBUG_PRINT(type);
        DEBUG_PRINT(")");
        DEBUG_PRINT(data);
        DEBUG_PRINT(",");
#endif
    }
#endif
}

float *Bn220::latP()
{
    return &lat_;
}

float *Bn220::lonP()
{
    return &lon_;
}

float *Bn220::altP()
{
    return &alt_;
}

float *Bn220::spdP()
{
    return &spd_;
}

float *Bn220::cogP()
{
    return &cog_;
}

float *Bn220::kphP()
{
    return &lat_;
}

uint8_t *Bn220::satP()
{
    return &sat_;
}

float *Bn220::dateP()
{
    return &date_;
}

float *Bn220::timeP()
{
    return &time_;
}