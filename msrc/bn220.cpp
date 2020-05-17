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
            break;
        case '\r':
        case ',':
            if (contIndex_ == 0)
            {
                if (strcmp(buffer_ + 2, "GGA") == 0)
                {
                    nmeaCmd_ = BN220_GGA;
                }
                else if (strcmp(buffer_ + 2, "GLL") == 0)
                {
                    nmeaCmd_ = BN220_GLL;
                }
                else if (strcmp(buffer_ + 2, "GMC") == 0)
                {
                    nmeaCmd_ = BN220_RMC;
                }
                else if (strcmp(buffer_ + 2, "VTG") == 0)
                {
                    nmeaCmd_ = BN220_VTG;
                }
                Serial.print("\n");
                Serial.print(nmeaCmd_);
                Serial.print(": ");
            }
            else
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
            contIndex_++;
            contBuff_ = 0;
            buffer_[0] = 0;
            break;
        case '\n':
            break;
        default:
            buffer_[contBuff_] = c;
            contBuff_++;
            buffer_[contBuff_] = 0;
        }
    }
}

void Bn220Interface::parser(uint8_t type, char *data)
{
    if (strlen(data))
    {
        Serial.print(type);
        Serial.print("(");
        Serial.print(data);
        Serial.print(") ");
        switch (type)
        {
        case BN220_TIME:
            time_ = atoi(data);
            break;
        case BN220_LAT: // deg, 2dec
            uint8_t deg;
            float min;
            strncpy(deg, data, 2);
            min = atof(data + 2);
            lat_ = deg + min / 60;
            break;
        case BN220_LON:
            strncpy(deg, data, 2);
            min = atof(data + 2);
            lon_ = deg + min / 60;
            break;
        case BN220_ALT:
            alt_ = atof(data);
            break;
        case BN220_SPD:
            spd_ = atof(data);
            break;
        case BN220_COG:
            cog_ = atof(data);
            break;
        case BN220_DATE:
            date_ = atoi(data);
            break;
        case BN220_KPH:
            kph_ = atof(data);
            break;
        case BN220_SAT:
            sat_ = atoi(data);
            break;
        }
    }
}

float Bn220Interface::read(uint8_t index)
{
}