#ifndef FORMATDATA_H
#define FORMATDATA_H

#define ALT_FIRST_ID 0x0100 // 100 m
#define ALT_LAST_ID 0x010f
#define VARIO_FIRST_ID 0x0110 // 100 m/s
#define VARIO_LAST_ID 0x011f
#define CURR_FIRST_ID 0x0200 // 10 A
#define CURR_LAST_ID 0x020f
#define VFAS_FIRST_ID 0x0210 // 100 v
#define VFAS_LAST_ID 0x021f
#define CELLS_FIRST_ID 0x0300 //
#define CELLS_LAST_ID 0x030f
#define T1_FIRST_ID 0x0400 // 1 C
#define T1_LAST_ID 0x040f
#define T2_FIRST_ID 0x0410 // 1 C
#define T2_LAST_ID 0x041f
#define RPM_FIRST_ID 0x0500 // 1 rpm
#define RPM_LAST_ID 0x050f
#define FUEL_FIRST_ID 0x0600 // 1 %
#define FUEL_LAST_ID 0x060f
#define ACCX_FIRST_ID 0x0700 // 100 g
#define ACCX_LAST_ID 0x070f
#define ACCY_FIRST_ID 0x0710 // 100 g
#define ACCY_LAST_ID 0x071f
#define ACCZ_FIRST_ID 0x0720 // 100 g
#define ACCZ_LAST_ID 0x072f
#define GPS_LONG_LATI_FIRST_ID 0x0800 // bit32(1<<31)=1=LON:=0=LAT, bit31(1<<30)=1=-:=0=+, escaler: 10000
#define GPS_LONG_LATI_LAST_ID 0x080f
#define GPS_ALT_FIRST_ID 0x0820 // 100 m
#define GPS_ALT_LAST_ID 0x082f
#define GPS_SPEED_FIRST_ID 0x0830 // 1000 kts
#define GPS_SPEED_LAST_ID 0x083f
#define GPS_COURS_FIRST_ID 0x0840 // 100 º
#define GPS_COURS_LAST_ID 0x084f
#define GPS_TIME_DATE_FIRST_ID 0x0850 // Date: Y M D 0xFF or Time: H M S 0x00
#define GPS_TIME_DATE_LAST_ID 0x085f
#define A3_FIRST_ID 0x0900 // 100 v
#define A3_LAST_ID 0x090f
#define A4_FIRST_ID 0x0910 // 100 v
#define A4_LAST_ID 0x091f
#define AIR_SPEED_FIRST_ID 0x0a00 // 10 kts
#define AIR_SPEED_LAST_ID 0x0a0f
#define RBOX_BATT1_FIRST_ID 0x0b00 // 1000 v, 100 A
#define RBOX_BATT1_LAST_ID 0x0b0f
#define RBOX_BATT2_FIRST_ID 0x0b10 // 1000 v, 100 A
#define RBOX_BATT2_LAST_ID 0x0b1f
#define RBOX_STATE_FIRST_ID 0x0b20 // 1
#define RBOX_STATE_LAST_ID 0x0b2f
#define RBOX_CNSP_FIRST_ID 0x0b30 // 1 mAh (1), 1mAh (2)
#define RBOX_CNSP_LAST_ID 0x0b3f
#define SD1_FIRST_ID 0x0b40
#define SD1_LAST_ID 0x0b4f
#define ESC_POWER_FIRST_ID 0x0b50 // bytes 1,2: 100 V,  bytes 3,4: 100 A
#define ESC_POWER_LAST_ID 0x0b5f
#define ESC_RPM_CONS_FIRST_ID 0x0b60 // bytes 1,2: 0.01 rpm,  bytes 3,4: 1 mah
#define ESC_RPM_CONS_LAST_ID 0x0b6f
#define ESC_TEMPERATURE_FIRST_ID 0x0b70 // 1 C
#define ESC_TEMPERATURE_LAST_ID 0x0b7f
#define X8R_FIRST_ID 0x0c20
#define X8R_LAST_ID 0x0c2f
#define S6R_FIRST_ID 0x0c30
#define S6R_LAST_ID 0x0c3f
#define GASSUIT_TEMP1_FIRST_ID 0x0d00 // 1 C
#define GASSUIT_TEMP1_LAST_ID 0x0d0f
#define GASSUIT_TEMP2_FIRST_ID 0x0d10 // 1 C
#define GASSUIT_TEMP2_LAST_ID 0x0d1f
#define GASSUIT_SPEED_FIRST_ID 0x0d20 // 1 rpm
#define GASSUIT_SPEED_LAST_ID 0x0d2f
#define GASSUIT_RES_VOL_FIRST_ID 0x0d30 // 1 ml
#define GASSUIT_RES_VOL_LAST_ID 0x0d3f
#define GASSUIT_RES_PERC_FIRST_ID 0x0d40 // 1 %
#define GASSUIT_RES_PERC_LAST_ID 0x0d4f
#define GASSUIT_FLOW_FIRST_ID 0x0d50 // 1 ml
#define GASSUIT_FLOW_LAST_ID 0x0d5f
#define GASSUIT_MAX_FLOW_FIRST_ID 0x0d60 // 1 ml
#define GASSUIT_MAX_FLOW_LAST_ID 0x0d6f
#define GASSUIT_AVG_FLOW_FIRST_ID 0x0d70 // 1 ml
#define GASSUIT_AVG_FLOW_LAST_ID 0x0d7f
#define SBEC_POWER_FIRST_ID 0x0e50 // bytes 1,2: 100 V,  bytes 3,4: 100 A
#define SBEC_POWER_LAST_ID 0x0e5f
#define DIY_FIRST_ID 0x5100
#define DIY_LAST_ID 0x52ff
#define DIY_STREAM_FIRST_ID 0x5000
#define DIY_STREAM_LAST_ID 0x50ff
#define FACT_TEST_ID 0xf000
#define RSSI_ID 0xf101
#define A1_ID 0xf102 // 10 v
#define A2_ID 0xf103 // 10 v
#define SP2UART_A_ID 0xfd00
#define SP2UART_B_ID 0xfd01
#define RXBT_ID 0xf104 // 10 v
#define RAS_ID 0xf105
#define XJT_VERSION_ID 0xf106
#define FUEL_QTY_FIRST_ID 0x0a10 // 100 ml
#define FUEL_QTY_LAST_ID 0x0a1f

#define TYPE_LAT 0
#define TYPE_LON 1
#define TYPE_DATE 0
#define TYPE_TIME 1

#include <Arduino.h>

class FormatData
{
protected:
    uint32_t formatData(uint16_t dataId, float valueL, float valueM);
    uint32_t formatData(uint16_t dataId, float valueL);
    uint32_t formatLatLon(uint8_t type, float value);
    uint32_t formatDateTime(uint8_t type, uint32_t value);
    uint32_t formatCell(uint8_t cellIndex, float value1, float value2);
};

#endif