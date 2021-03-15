#include "formatData.h"

uint32_t FormatData::formatData(uint16_t dataId, float valueL, float valueM)
{
    if (dataId >= ESC_POWER_FIRST_ID && dataId <= ESC_POWER_LAST_ID)
        return (uint32_t)round(valueM * 100) << 16 | (uint16_t)round(valueL * 100);

    if (dataId >= ESC_RPM_CONS_FIRST_ID && dataId <= ESC_RPM_CONS_LAST_ID)
    {
        return (uint32_t)round(valueM) << 16 | (uint16_t)round((valueL) / 100);
    }

    if (dataId >= SBEC_POWER_FIRST_ID && dataId <= SBEC_POWER_LAST_ID)
        return (uint32_t)round(valueM * 1000) << 16 | (uint16_t)round((valueL)*1000);

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
        return value;

    if (dataId == GPS_ALT_AP_ID ||
        dataId == BARO_ALT_AP_ID ||
        dataId == GPS_SPEED_AP_ID ||
        dataId == GPS_LONG_AP_ID ||
        dataId == GPS_LAT_AP_ID ||
        dataId == GPS_COURS_AP_ID)
        return (value - (int16_t)value) * 10000;

    if (dataId == VOLTS_BP_ID)
        return value * 2;

    if (dataId == VOLTS_AP_ID)
        return ((value * 2) - (int16_t)(value * 2)) * 10000;

    if (dataId == GPS_LONG_BP_ID || dataId == GPS_LAT_BP_ID)
    {
        uint8_t deg = value / 60;
        uint8_t min = (int)value % 60;
        char buf[6];
        sprintf(buf, "%d%d", deg, min);
        return atoi(buf);
    }

    if (dataId == GPS_YEAR_ID)
    {
        return value / 10000;
    }

    if (dataId == GPS_DAY_MONTH_ID)
    {
        return value - (int16_t)(value / 10000);
    }

    if (dataId == GPS_HOUR_MIN_ID)
    {
        return value / 100;
    }

    if (dataId == GPS_SEC_ID)
    {
        return value - (int16_t)(value / 100);
    } 

    if (dataId == CURRENT_ID || dataId == VFAS_ID)
        return round(value * 10);

    return round(value);
}