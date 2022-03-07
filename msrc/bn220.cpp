#include "bn220.h"

Bn220::Bn220(AbstractSerial &serial, uint32_t baud) : serial_(serial), baud_(baud)
{
}

void Bn220::begin()
{
    serial_.begin(baud_, SERIAL__8N1);
}

void Bn220::update()
{
    while (serial_.available())
    {
        static uint8_t contBuff = 0;
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
                else if (memcmp(buffer_ + 2, "RMC", 3) == 0)
                {
                    nmeaCmd_ = BN220_RMC;
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
                if (nmeaCmd_ == BN220_GGA)
                    processField(gga);
                else if (nmeaCmd_ == BN220_RMC)
                    processField(rmc);
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
    lat_ = -692.761166667;   // 11º32'45.67" +N, -S
    lon_ = -1251.964833333;  // 20º51'57.89" +E, -W
    alt_ = 1283;   // m
    spd_ = 158;    // kts
    cog_ = 123.45; // º
    sat_ = 10;     //
    date_ = 141012; // yymmdd
    time_ = 162302; // hhmmss
    hdop_ = 12.35; //
    vario_ = 5.67; // m/s
#else
    vario_ = calcSpeed(alt_, 2000);
    //dist_ = calcDistanceToHome(lat_ / 60, lon_ / 60, alt_, 1000);
#endif
}

float Bn220::degreesToRadians(float degrees)
{
    return degrees * PI / 180;
}

float Bn220::calcDistanceToHome(float lat, float lon, float alt, uint16_t intervalMin)
{
    static uint16_t msPrev = 0;
    static float distance = 10000;
    static float latInit = 0;
    static float lonInit = 0;
    static float dLatInit = 0;
    static float altInit = 0;
    if ((uint16_t)(millis() - msPrev) > intervalMin && sat_ > 9)
    {
        if (latInit == 0 && lonInit == 0)
        {
            latInit = lat;
            lonInit = lon;
            dLatInit = degreesToRadians(latInit);
            altInit = alt;
        }
        uint16_t earthRadiusKm = 6371;
        float dLatDelta = degreesToRadians(latInit - lat);
        float dLonDelta = degreesToRadians(lonInit - lon);
        float dLat = degreesToRadians(lat);
        float a = sin(dLatDelta / 2) * sin(dLatDelta / 2) + sin(dLonDelta / 2) * sin(dLonDelta / 2) * cos(dLat) * cos(dLatInit);
        float c = 2 * atan2(sqrt(a), sqrt(1 - a));
        float distanceOnGound = earthRadiusKm * c * 1000;
        distance = sqrt(distanceOnGound * distanceOnGound + (alt-altInit) * (alt-altInit)); // meters
        msPrev = millis();
    }
    return distance;
}

void Bn220::processField(const uint8_t *command)
{
    uint8_t i = 0;
    while (command[i] != BN220_END)
    {
        if (i == contIndex - 1 && command[i])
        {
            parser(command[i], buffer_);
        }
        i++;
    }
}

void Bn220::parser(uint8_t type, char *data)
{
    if (strlen(data))
    {
        if (type == BN220_TIME)
        {
            time_ = atof(data);
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
            date_ = atof(data);
        }
        else if (type == BN220_SAT)
        {
            sat_ = atof(data);
        }
        else if (type == BN220_LAT_SIGN)
        {
            (data[0] == 'N') ? latDir_ = 1 : latDir_ = -1;
        }
        else if (type == BN220_LON_SIGN)
        {
            (data[0] == 'E') ? lonDir_ = 1 : lonDir_ = -1;
        }
        else if (type == BN220_HDOP)
        {
            hdop_ = atof(data);
        }
#ifdef DEBUG_GPS
        DEBUG_PRINT("(");
        DEBUG_PRINT(type);
        DEBUG_PRINT(")");
        DEBUG_PRINT(data);
        DEBUG_PRINT(",");
#endif
    }
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

float *Bn220::satP()
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

float *Bn220::hdopP()
{
    return &hdop_;
}

float *Bn220::varioP()
{
    return &vario_;
}

float *Bn220::distP()
{
    return &dist_;
}