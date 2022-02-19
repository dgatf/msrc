#include "formatData.h"

uint32_t FormatData::formatData(uint16_t dataId, float valueL, float valueM)
{
    if ((dataId >= ESC_POWER_FIRST_ID && dataId <= ESC_POWER_LAST_ID) ||
        (dataId >= SBEC_POWER_FIRST_ID && dataId <= SBEC_POWER_LAST_ID))
        return (uint32_t)round(valueM * 100) << 16 | (uint16_t)round(valueL * 100);

    if (dataId >= ESC_RPM_CONS_FIRST_ID && dataId <= ESC_RPM_CONS_LAST_ID)
    {
        return (uint32_t)round(valueM) << 16 | (uint16_t)round((valueL) / 100);
    }

    //if (dataId >= CELLS_FIRST_ID && dataId <= CELLS_LAST_ID)
    return (uint16_t)round(valueM * 500) << 8 | (uint16_t)valueL;
}

uint32_t FormatData::formatData(uint16_t dataId, float valueL)
{
    if ((dataId >= GPS_SPEED_FIRST_ID && dataId <= GPS_SPEED_LAST_ID) ||
        (dataId >= RBOX_BATT1_FIRST_ID && dataId <= RBOX_BATT2_FIRST_ID))
        return round(valueL * 1000);

    if ((dataId >= ALT_FIRST_ID && dataId <= VARIO_LAST_ID) ||
        (dataId >= VFAS_FIRST_ID && dataId <= VFAS_LAST_ID) ||
        (dataId >= ACCX_FIRST_ID && dataId <= GPS_ALT_LAST_ID) ||
        (dataId >= GPS_COURS_FIRST_ID && dataId <= GPS_COURS_LAST_ID) ||
        (dataId >= A3_FIRST_ID && dataId <= A4_LAST_ID))
        return round(valueL * 100);

    if ((dataId >= CURR_FIRST_ID && dataId <= CURR_LAST_ID) ||
        (dataId >= AIR_SPEED_FIRST_ID && dataId <= AIR_SPEED_LAST_ID) ||
        dataId == A1_ID || dataId == A2_ID || dataId == RXBT_ID)
        return round(valueL * 10);

    return round(valueL);
}

uint32_t FormatData::formatCell(uint8_t cellIndex, float value1, float value2)
{
    return cellIndex | (uint16_t)round(value1 * 500) << 8 | (uint32_t)round(value2 * 500) << 22;
}

uint32_t FormatData::formatLatLon(uint8_t type, float value)
{
    uint32_t data = 0;
    if (value < 0)
        data |= (uint32_t)1 << 30;
    if (type == TYPE_LON)
    {
        data |= (uint32_t)1 << 31;
    }
    data |= (uint32_t)abs(round(value * 10000));
    return data;
}

uint32_t FormatData::formatDateTime(uint8_t type, uint32_t value)
{
    uint8_t dayHour = value / 10000;
    uint8_t monthMin = value / 100 - dayHour * 100;
    uint8_t yearSec = value - (value / 100) * 100;
    if (type == TYPE_DATE)
    {
        return (uint32_t)yearSec << 24 | (uint32_t)monthMin << 16 | dayHour << 8 | 0xFF;
    }
    return (uint32_t)dayHour << 24 | (uint32_t)monthMin << 16 | yearSec << 8;
}

uint16_t FormatData::formatData(uint8_t dataId, float value)
{
    if (dataId == GPS_ALT_BP_ID ||
        dataId == BARO_ALT_BP_ID ||
        dataId == GPS_SPEED_BP_ID ||
        dataId == GPS_COURS_BP_ID)
        return (int16_t)value;

    if (dataId == GPS_ALT_AP_ID ||
        dataId == BARO_ALT_AP_ID ||
        dataId == GPS_SPEED_AP_ID ||
        dataId == GPS_LONG_AP_ID ||
        dataId == GPS_LAT_AP_ID ||
        dataId == GPS_COURS_AP_ID)
        return (abs(value) - (int16_t)abs(value)) * 10000;

    if (dataId == VOLTS_BP_ID)
        return value * 2;

    if (dataId == VOLTS_AP_ID)
        return ((value * 2) - (int16_t)(value * 2)) * 10000;

    if (dataId == GPS_LONG_BP_ID || dataId == GPS_LAT_BP_ID)
    {
        value = abs(value);
        uint8_t deg = value / 60;
        uint8_t min = (int)value % 60;
        char buf[6];
        sprintf(buf, "%d%d", deg, min);
        return atoi(buf);
    }

    if (dataId == GPS_LONG_EW_ID)
    {
        if (value >= 0)
            return 'E';
        return 'O';
    }

    if (dataId == GPS_LAT_NS_ID)
    {
        if (value >= 0)
            return 'N';
        return 'S';
    }

    if (dataId == GPS_YEAR_ID)
    {
        return value / 10000;
    }

    if (dataId == GPS_DAY_MONTH_ID)
    {
        return value - (uint32_t)(value / 10000) * 10000;
    }

    if (dataId == GPS_HOUR_MIN_ID)
    {
        return value / 100;
    }

    if (dataId == GPS_SEC_ID)
    {
        return value - (uint32_t)(value / 100) * 100;
    }

    if (dataId == CURRENT_ID || dataId == VFAS_ID)
        return round(value * 10);

    if (dataId == RPM_ID)
        return value / 60;

    return round(value);
}

int32_t FormatData::formatIbus(uint8_t dataId, float value)
{

    if (dataId == AFHDS2A_ID_TEMPERATURE)
        return round((value + 40) * 10);

    if (dataId == AFHDS2A_ID_EXTV ||
        dataId == AFHDS2A_ID_CELL_VOLTAGE ||
        dataId == AFHDS2A_ID_BAT_CURR ||
        dataId == AFHDS2A_ID_CLIMB_RATE ||
        dataId == AFHDS2A_ID_COG ||
        dataId == AFHDS2A_ID_VERTICAL_SPEED ||
        dataId == AFHDS2A_ID_GROUND_SPEED ||
        dataId == AFHDS2A_ID_GPS_ALT ||
        dataId == AFHDS2A_ID_PRES ||
        dataId == AFHDS2A_ID_ALT)
        return round(value * 100);

    if (dataId == AFHDS2A_ID_GPS_LAT ||
        dataId == AFHDS2A_ID_GPS_LON)
        return round(value / 60 * 1E7);

    if (dataId == AFHDS2A_ID_SPE)
        return round(value * 100 * 1.852);

    if (dataId == AFHDS2A_ID_GPS_STATUS)
        return value * 256;
    
    if (dataId == AFHDS2A_ID_S84 ||
        dataId == AFHDS2A_ID_S85)
        return value * 1e5;

    return round(value);
}

uint16_t FormatData::formatSbus(uint8_t dataId, float value)
{
    if (dataId == FASST_RPM)
    {
        return __builtin_bswap16((uint16_t)round(value / 6));
    }
    if (dataId == FASST_TEMP)
    {
        return __builtin_bswap16((uint16_t)round(value + 100) | 0X8000);
    }
    if (dataId == FASST_VOLT_V1)
    {
        return __builtin_bswap16((uint16_t)round(value * 10) | 0x8000);
    }
    if (dataId == FASST_VOLT_V2)
    {
        return __builtin_bswap16((uint16_t)round(value * 10));
    }
    if (dataId == FASST_VARIO_SPEED)
    {
        return __builtin_bswap16((int16_t)round(value * 100));
    }
    if (dataId == FASST_VARIO_ALT)
    {
        return __builtin_bswap16((int16_t)round(value) | 0x4000);
    }
    if (dataId == FASST_POWER_CURR)
    {
        return __builtin_bswap16((uint16_t)round(value * 100) | 0x4000);
    }
    if (dataId == FASST_POWER_VOLT)
    {
        return __builtin_bswap16((uint16_t)round(value * 100));
    }
    if (dataId == FASST_GPS_SPEED)
    {
        return __builtin_bswap16((uint16_t)round(value) | 0x4000);
    }
    if (dataId == FASST_GPS_VARIO_SPEED)
    {
        return __builtin_bswap16((int16_t)round(value) * 10 | 0x4000);
    }
    if (dataId == FASST_GPS_ALTITUDE)
    {
        return __builtin_bswap16((int16_t)round(value) | 0x4000);
    }
    if (dataId == FASST_GPS_LATITUDE1 || dataId == FASST_GPS_LONGITUDE1)
    {
        // FFFF = (deg,deg,S/W,min) -> min *10000 (prec 4)
        uint16_t lat;
        if (value < 0)
        {
            lat = 1 << FASST_SOUTH_WEST_BIT;
            value *= -1;
        }
        uint8_t degrees = value / 60;
        lat |= degrees << 8;
        uint32_t minutes = fmod(value, 60) * 10000; // minutes precision 4
        lat |= minutes >> 16;
        return __builtin_bswap16(lat);
    }
    if (dataId == FASST_GPS_LATITUDE2 || dataId == FASST_GPS_LONGITUDE2)
    {
        // FFFF = (min) -> min *10000 (prec 4)
        if (value < 0)
        {
            value *= -1;
        }
        uint32_t minutes = fmod(value, 60) * 10000; // minutes precision 4
        return __builtin_bswap16(minutes);
    }
    if (dataId == FASST_GPS_TIME)
    {
        if (value > 120000)
            value -= 120000;
        uint8_t hours = value / 10000;
        uint8_t minutes = (uint8_t)(value / 100) - hours * 100;
        uint8_t seconds = (uint8_t)(value / 10000);
        return __builtin_bswap16(hours * 3600 + minutes * 60 + seconds);
    }
    return __builtin_bswap16(round(value));
}

int16_t FormatData::formatMultiplex(uint8_t dataId, float value)
{
    int16_t formatted;
    if (dataId == FHSS_VOLTAGE ||
        dataId == FHSS_CURRENT ||
        dataId == FHSS_VARIO ||
        dataId == FHSS_SPEED ||
        dataId == FHSS_TEMP ||
        dataId == FHSS_COURSE ||
        dataId == FHSS_DISTANCE)
        formatted = round(value * 10);
    else if (dataId == FHSS_RPM)
        formatted = round(value / 10);
    else
        formatted = round(value);
    if (formatted > 16383)
        formatted = 16383;
    if (formatted < -16383)
        formatted = -16383;
    bool isNegative = false;
    if (formatted < 0)
        isNegative = true;
    formatted <<= 1;
    if (isNegative)
        formatted |= 1 << 15;
    return formatted;
}